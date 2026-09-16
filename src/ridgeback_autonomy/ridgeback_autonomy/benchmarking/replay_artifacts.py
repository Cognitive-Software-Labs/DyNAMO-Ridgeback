"""Typed immutable artifacts for layered replay.

Legacy ``schema_version: 1`` measurement datasets remain owned by
``benchmarking.replay``.  This module owns the version-2 envelope, full sensor
captures, derived mask caches, payload hashing, and exact parent lineage.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import shutil
from typing import Any, Iterable
import uuid

import numpy as np

from ridgeback_autonomy.perception.target_localization.core.mask import (
    MaskPrecision,
    MaskRegion,
)
from ridgeback_autonomy.perception.target_localization.estimator_registry import (
    parse_estimators,
)


FORMAT_NAME = 'dynamo-replay'
MANIFEST_VERSION = 2
MANIFEST_NAME = 'manifest.json'
SENSOR_CAPTURE_KIND = 'sensor-capture'
MASK_CACHE_KIND = 'mask-cache'
# Version 2 adds the LiDAR scan and its scan -> camera extrinsic per event, so
# polar profiling can be measured off frozen evidence. Version 1 captures stay
# readable with the scan simply absent: mask caches are parented to a specific
# sensor capture by hash, and rejecting the parent would strand every cache
# derived from it.
SENSOR_PAYLOAD_VERSION = 2
SENSOR_PAYLOAD_VERSIONS = (1, 2)
# The first payload version that can carry a scan at all. Read this rather than
# comparing against the current version: what decides whether polar profiling is
# reachable on a capture is when scan recording began, not what the writer emits
# today, and those two stop being the same number at version 3.
SENSOR_SCAN_PAYLOAD_VERSION = 2
MASK_PAYLOAD_VERSION = 1

# The ``sensor_msgs/LaserScan`` geometry a stored scan must carry. ``ranges``
# travels beside them as an npz array; these five are what turns a beam index
# into a bearing and declare which returns the driver considers usable.
SCAN_GEOMETRY_KEYS = (
    'angle_min', 'angle_max', 'angle_increment', 'range_min', 'range_max')

MASK_STATUS_OK = 'ok'
MASK_STATUS_OVERSIZED_BOX = 'oversized-box'
MASK_STATUS_EMPTY_SEGMENTATION = 'empty-segmentation'
MASK_STATUS_NO_COLOR_FRAME = 'no-color-frame'
MASK_STATUSES = frozenset({
    MASK_STATUS_OK,
    MASK_STATUS_OVERSIZED_BOX,
    MASK_STATUS_EMPTY_SEGMENTATION,
    MASK_STATUS_NO_COLOR_FRAME,
})


def canonical_json_bytes(value: Any) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(',', ':'), ensure_ascii=False,
    ).encode('utf-8')


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _write_json_atomic(path: Path, document: dict) -> None:
    temporary = path.with_name(f'.{path.name}.partial')
    with open(temporary, 'w', encoding='utf-8') as handle:
        json.dump(document, handle, indent=2, sort_keys=True)
        handle.write('\n')
    os.replace(temporary, path)


def _artifact_identity(manifest: dict) -> str:
    """Identity of frozen evidence, excluding location/runtime annotations."""

    artifact = manifest.get('artifact', {})
    content = {
        'format': manifest.get('format'),
        'manifest_version': manifest.get('manifest_version'),
        'kind': artifact.get('kind'),
        'payload_version': artifact.get('payload_version'),
        'parent': manifest.get('parent'),
        'producer_signature_sha256': manifest.get('producer', {}).get(
            'signature_sha256'),
        'trials': [
            {
                key: entry.get(key)
                for key in (
                    'trial_id', 'sha256', 'event_count', 'detection_count')
            }
            for entry in manifest.get('trials', [])
        ],
    }
    return sha256_bytes(canonical_json_bytes(content))


def producer_signature(producer: dict) -> str:
    return sha256_bytes(canonical_json_bytes(producer))


def dependency_versions(names: Iterable[str]) -> dict[str, str | None]:
    """Resolve package versions without importing the packages themselves."""

    versions: dict[str, str | None] = {}
    for name in names:
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
    return versions


def _scalar_string(array: np.ndarray) -> str:
    return str(np.asarray(array).item())


@dataclass(frozen=True)
class ReplayArtifact:
    root: Path
    manifest: dict
    manifest_sha256: str

    @property
    def kind(self) -> str:
        return str(self.manifest['artifact']['kind'])

    @property
    def id(self) -> str:
        return str(self.manifest['artifact']['id'])

    @property
    def payload_version(self) -> int:
        return int(self.manifest['artifact']['payload_version'])

    @property
    def trial_entries(self) -> tuple[dict, ...]:
        return tuple(self.manifest['trials'])


@dataclass(frozen=True)
class CachedMaskOutcome:
    """One index-aligned materializer result for one parent detection."""

    region: MaskRegion | None
    status: str
    duration_ms: float | None = None

    def __post_init__(self) -> None:
        if self.status not in MASK_STATUSES:
            raise ValueError(f'Unknown mask producer status {self.status!r}.')
        if self.status == MASK_STATUS_OK and self.region is None:
            raise ValueError('Mask status "ok" requires a MaskRegion.')
        if self.status != MASK_STATUS_OK and self.region is not None:
            raise ValueError(f'Mask status {self.status!r} cannot carry a region.')


class _ArtifactWriter:
    def __init__(
        self,
        root: str | Path,
        *,
        kind: str,
        payload_version: int,
        producer: dict,
        parent: dict | None = None,
        metadata: dict | None = None,
    ) -> None:
        self.root = Path(root).expanduser().resolve()
        if self.root.exists():
            raise ValueError(f'Replay artifact directory already exists: {self.root}')
        self.root.parent.mkdir(parents=True, exist_ok=True)
        self.temporary_root = self.root.with_name(
            f'.{self.root.name}.partial-{os.getpid()}-{uuid.uuid4().hex}')
        self.temporary_root.mkdir()
        self.trials_dir = self.temporary_root / 'trials'
        self.trials_dir.mkdir()
        normalized_producer = deepcopy(producer)
        normalized_producer['signature_sha256'] = producer_signature(producer)
        self.manifest = {
            'format': FORMAT_NAME,
            'manifest_version': MANIFEST_VERSION,
            'artifact': {
                'id': '',
                'kind': kind,
                'state': 'capturing',
                'payload_version': payload_version,
            },
            'producer': normalized_producer,
            'trials': [],
        }
        if parent is not None:
            self.manifest['parent'] = deepcopy(parent)
        if metadata:
            self.manifest['metadata'] = deepcopy(metadata)
        _write_json_atomic(self.temporary_root / MANIFEST_NAME, self.manifest)

    def _write_payload(
        self,
        *,
        trial_id: str,
        arrays: dict[str, np.ndarray],
        event_count: int,
        detection_count: int,
    ) -> None:
        if not trial_id or trial_id in ('.', '..') or Path(trial_id).name != trial_id:
            raise ValueError(f'Replay trial id is not path-safe: {trial_id!r}.')
        relative = f'trials/{trial_id}.npz'
        payload_path = self.temporary_root / relative
        if payload_path.exists() or any(
            entry['trial_id'] == trial_id for entry in self.manifest['trials']
        ):
            raise ValueError(f'Replay artifact already contains trial "{trial_id}".')
        temporary = payload_path.with_name(f'.{payload_path.name}.partial.npz')
        np.savez_compressed(temporary, **arrays)
        os.replace(temporary, payload_path)
        self.manifest['trials'].append({
            'trial_id': trial_id,
            'payload': relative,
            'sha256': sha256_file(payload_path),
            'event_count': int(event_count),
            'detection_count': int(detection_count),
        })
        _write_json_atomic(self.temporary_root / MANIFEST_NAME, self.manifest)

    def _close(self, state: str, **metadata: Any) -> ReplayArtifact | None:
        if not self.manifest['trials']:
            raise ValueError('Cannot close a replay artifact without any trial payloads.')
        if metadata:
            self.manifest.setdefault('metadata', {}).update(metadata)
        self.manifest['artifact']['state'] = state
        if state == 'complete':
            self.manifest['artifact']['id'] = _artifact_identity(self.manifest)
        _write_json_atomic(self.temporary_root / MANIFEST_NAME, self.manifest)
        os.replace(self.temporary_root, self.root)
        if state != 'complete':
            return None
        return load_artifact(self.root, require_parent=False)

    def finalize(self, **metadata: Any) -> ReplayArtifact:
        artifact = self._close('complete', **metadata)
        assert artifact is not None
        return artifact

    def mark_incomplete(self, **metadata: Any) -> None:
        self._close('incomplete', **metadata)

    def discard_empty(self) -> None:
        """Remove an unexposed temporary directory when setup fails pre-payload."""

        if self.temporary_root.exists() and not self.manifest['trials']:
            shutil.rmtree(self.temporary_root)


def validate_scan_record(scan: dict | None) -> dict | None:
    """Normalize one event's scan record, or ``None`` when no scan was captured.

    Shared by both evidence formats. The scan contract is a property of what a
    ``LaserScan`` must carry to be replayable, not of the file it ends up in, so
    the sensor capture and the legacy dataset check it with the same function
    rather than drifting apart on which fields are required.

    Shape-checked the way RGB and depth are, and for the same reason: beam
    position encodes bearing order, which the run segmentation depends on, so a
    2-D or empty ranges payload has no profile to segment. The five geometry
    scalars must all be present because without them the beams cannot be
    projected at all.

    Their VALUES are not second-guessed here. ``polar_profiling`` applies
    ``range_min`` / ``range_max`` exactly as the driver declared them and
    handles an unusable cap itself, so narrowing them at capture time would
    silently discard returns the estimator is entitled to see.
    """

    if scan is None:
        return None
    if 'ranges' not in scan:
        raise ValueError('Scan record carries no ranges array.')
    missing = tuple(
        key for key in (*SCAN_GEOMETRY_KEYS, 'frame_id', 'stamp_ns')
        if scan.get(key) is None)
    if missing:
        raise ValueError(f'Scan record is missing {", ".join(missing)}.')
    ranges = np.asarray(scan['ranges'], dtype=np.float32)
    if ranges.ndim != 1 or ranges.size == 0:
        raise ValueError(
            f'Scan ranges must be a non-empty 1-D array; got shape {ranges.shape}.')
    return {
        **{key: float(scan[key]) for key in SCAN_GEOMETRY_KEYS},
        'ranges': np.ascontiguousarray(ranges),
        'frame_id': str(scan['frame_id']),
        'stamp_ns': int(scan['stamp_ns']),
    }


def _validate_sensor_event(
    event: dict,
) -> tuple[np.ndarray | None, np.ndarray | None, dict | None]:
    height, width = int(event['image_height']), int(event['image_width'])
    detections = event.get('detections')
    if not isinstance(detections, list):
        raise ValueError('Sensor event detections must be a list.')
    if int(event.get('count', -1)) != len(detections):
        raise ValueError('Sensor event count differs from its detections list.')
    if bool(event.get('detected')) != bool(detections):
        raise ValueError('Sensor event detected flag differs from its detections list.')

    rgb = event.get('rgb')
    if rgb is not None:
        rgb = np.asarray(rgb)
        if rgb.dtype != np.uint8 or rgb.shape != (height, width, 3):
            raise ValueError(
                f'RGB must be uint8 {(height, width, 3)}; got {rgb.dtype} {rgb.shape}.')
    depth = event.get('depth_m')
    if depth is not None:
        depth = np.asarray(depth)
        if depth.dtype != np.float32 or depth.shape != (height, width):
            raise ValueError(
                f'Depth must be float32 {(height, width)}; got {depth.dtype} {depth.shape}.')
    return rgb, depth, validate_scan_record(event.get('scan'))


class SensorCaptureWriter(_ArtifactWriter):
    """Write full-grid RGB/depth/context evidence, one compressed trial payload."""

    def __init__(
        self,
        root: str | Path,
        *,
        producer: dict,
        metadata: dict | None = None,
    ) -> None:
        super().__init__(
            root,
            kind=SENSOR_CAPTURE_KIND,
            payload_version=SENSOR_PAYLOAD_VERSION,
            producer=producer,
            metadata=metadata,
        )

    def write_trial(self, trial: dict, events: list[dict]) -> None:
        trial_id = str(trial['trial_id'])
        arrays: dict[str, np.ndarray] = {}
        serialized_events: list[dict] = []
        detection_count = 0
        for event_index, event in enumerate(events):
            rgb, depth, scan = _validate_sensor_event(event)
            item = {
                key: deepcopy(value)
                for key, value in event.items()
                if key not in ('rgb', 'depth_m', 'scan')
            }
            item['rgb_key'] = None
            item['depth_key'] = None
            item['scan_key'] = None
            item['scan'] = None
            if rgb is not None:
                item['rgb_key'] = f'rgb_{event_index}'
                arrays[item['rgb_key']] = np.ascontiguousarray(rgb)
            if depth is not None:
                item['depth_key'] = f'depth_{event_index}'
                arrays[item['depth_key']] = np.asarray(depth, dtype=np.float32)
            if scan is not None:
                item['scan_key'] = f'scan_{event_index}'
                arrays[item['scan_key']] = scan['ranges']
                item['scan'] = {
                    key: value for key, value in scan.items() if key != 'ranges'}
            serialized_events.append(item)
            detection_count += len(item['detections'])
        arrays['metadata_json'] = np.asarray(json.dumps({
            'payload_version': SENSOR_PAYLOAD_VERSION,
            'trial': trial,
            'events': serialized_events,
        }, sort_keys=True))
        self._write_payload(
            trial_id=trial_id,
            arrays=arrays,
            event_count=len(events),
            detection_count=detection_count,
        )


class MaskCacheWriter(_ArtifactWriter):
    """Write packed ``MaskRegion`` children with exact sensor-parent lineage."""

    def __init__(
        self,
        root: str | Path,
        *,
        parent: ReplayArtifact,
        producer: dict,
        metadata: dict | None = None,
    ) -> None:
        if parent.kind != SENSOR_CAPTURE_KIND:
            raise ValueError('A mask cache parent must be a sensor-capture artifact.')
        super().__init__(
            root,
            kind=MASK_CACHE_KIND,
            payload_version=MASK_PAYLOAD_VERSION,
            producer=producer,
            parent={
                'artifact_id': parent.id,
                'manifest_sha256': parent.manifest_sha256,
            },
            metadata=metadata,
        )

    def write_trial(
        self,
        trial_id: str,
        outcomes: list[list[CachedMaskOutcome]],
    ) -> None:
        arrays: dict[str, np.ndarray] = {}
        serialized_events: list[list[dict]] = []
        detection_count = 0
        for event_index, event_outcomes in enumerate(outcomes):
            serialized: list[dict] = []
            for detection_index, outcome in enumerate(event_outcomes):
                item: dict[str, Any] = {
                    'status': outcome.status,
                    'duration_ms': outcome.duration_ms,
                    'region': None,
                }
                if outcome.region is not None:
                    region = outcome.region
                    key = f'mask_{event_index}_{detection_index}'
                    flat = np.asarray(region.data, dtype=np.uint8).reshape(-1)
                    arrays[key] = np.packbits(flat, bitorder='little')
                    item['region'] = {
                        'data_key': key,
                        'shape': list(region.roi_shape),
                        'origin_u': region.origin_u,
                        'origin_v': region.origin_v,
                        'image_width': region.image_width,
                        'image_height': region.image_height,
                        'precision': region.precision.value,
                    }
                serialized.append(item)
                detection_count += 1
            serialized_events.append(serialized)
        arrays['metadata_json'] = np.asarray(json.dumps({
            'payload_version': MASK_PAYLOAD_VERSION,
            'trial_id': str(trial_id),
            'events': serialized_events,
        }, sort_keys=True))
        self._write_payload(
            trial_id=str(trial_id),
            arrays=arrays,
            event_count=len(outcomes),
            detection_count=detection_count,
        )


def _read_manifest(root: Path) -> tuple[dict, Path]:
    manifest_path = root / MANIFEST_NAME
    try:
        with open(manifest_path, encoding='utf-8') as handle:
            return json.load(handle), manifest_path
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f'Cannot read replay artifact manifest {manifest_path}: {exc}') from exc


def load_replay_input(
    path: str | Path,
    *,
    parent: ReplayArtifact | None = None,
):
    """Manifest dispatcher for legacy measurement and typed replay evidence."""

    root = Path(path).expanduser().resolve()
    manifest, manifest_path = _read_manifest(root)
    # Which FORMAT this is, not which version of it. A legacy manifest is the
    # one carrying a bare ``schema_version``; whether that version is readable is
    # the legacy loader's own question, and answering it here would have to be
    # restated every time that format gains a version -- and would report an
    # unsupported one as an unknown kind.
    if 'format' not in manifest and manifest.get('schema_version') is not None:
        from ridgeback_autonomy.benchmarking.replay import load_dataset
        return load_dataset(root)
    if manifest.get('format') == FORMAT_NAME:
        return load_artifact(root, parent=parent)
    raise ValueError(f'Unknown replay manifest kind or version: {manifest_path}')


def load_artifact(
    path: str | Path,
    *,
    parent: ReplayArtifact | None = None,
    require_parent: bool = True,
) -> ReplayArtifact:
    root = Path(path).expanduser().resolve()
    manifest, manifest_path = _read_manifest(root)
    if manifest.get('format') != FORMAT_NAME:
        raise ValueError(f'Unsupported replay artifact format in {manifest_path}.')
    if manifest.get('manifest_version') != MANIFEST_VERSION:
        raise ValueError(
            f'Unsupported replay manifest version {manifest.get("manifest_version")!r}; '
            f'expected {MANIFEST_VERSION}.')
    artifact_data = manifest.get('artifact')
    if not isinstance(artifact_data, dict):
        raise ValueError(f'Replay manifest has no artifact envelope: {manifest_path}')
    if artifact_data.get('state') != 'complete':
        raise ValueError(f'Replay artifact is not complete: {root}')
    producer = manifest.get('producer')
    if not isinstance(producer, dict):
        raise ValueError(f'Replay artifact has no producer provenance: {root}')
    unsigned_producer = dict(producer)
    declared_signature = unsigned_producer.pop('signature_sha256', None)
    if declared_signature != producer_signature(unsigned_producer):
        raise ValueError(f'Replay artifact producer signature mismatch: {manifest_path}')
    kind = artifact_data.get('kind')
    readable_payloads = {
        SENSOR_CAPTURE_KIND: SENSOR_PAYLOAD_VERSIONS,
        MASK_CACHE_KIND: (MASK_PAYLOAD_VERSION,),
    }.get(kind)
    if readable_payloads is None:
        raise ValueError(f'Unsupported replay artifact kind {kind!r}.')
    if artifact_data.get('payload_version') not in readable_payloads:
        raise ValueError(
            f'Unsupported {kind} payload version {artifact_data.get("payload_version")!r}.')
    expected_id = _artifact_identity(manifest)
    if artifact_data.get('id') != expected_id:
        raise ValueError(f'Replay artifact identity mismatch: {manifest_path}')
    trials = manifest.get('trials')
    if not isinstance(trials, list) or not trials:
        raise ValueError(f'Replay artifact {root} contains no trials.')
    seen: set[str] = set()
    for entry in trials:
        trial_id = str(entry.get('trial_id', ''))
        if not trial_id or trial_id in seen:
            raise ValueError(f'Replay artifact has a missing or duplicate trial id {trial_id!r}.')
        seen.add(trial_id)
        payload = (root / str(entry.get('payload', ''))).resolve()
        if not payload.is_relative_to(root):
            raise ValueError(f'Replay artifact payload escapes its root: {payload}')
        if not payload.is_file() or sha256_file(payload) != entry.get('sha256'):
            raise ValueError(f'Replay artifact payload hash mismatch: {payload}')
    artifact = ReplayArtifact(
        root=root,
        manifest=manifest,
        manifest_sha256=sha256_file(manifest_path),
    )
    if kind == MASK_CACHE_KIND:
        declared = manifest.get('parent')
        if not isinstance(declared, dict):
            raise ValueError(f'Mask cache has no declared parent: {root}')
        if parent is None and require_parent:
            raise ValueError(
                f'Mask cache requires its declared sensor-capture parent: {root}')
        if parent is not None:
            if parent.kind != SENSOR_CAPTURE_KIND:
                raise ValueError('Mask cache parent is not a sensor capture.')
            if declared.get('artifact_id') != parent.id:
                raise ValueError('Mask cache parent artifact id mismatch.')
            if declared.get('manifest_sha256') != parent.manifest_sha256:
                raise ValueError('Mask cache parent manifest hash mismatch.')
    return artifact


def load_sensor_trial(artifact: ReplayArtifact, entry: dict) -> tuple[dict, list[dict]]:
    if artifact.kind != SENSOR_CAPTURE_KIND:
        raise ValueError('load_sensor_trial requires a sensor-capture artifact.')
    path = artifact.root / entry['payload']
    try:
        with np.load(path, allow_pickle=False) as payload:
            metadata = json.loads(_scalar_string(payload['metadata_json']))
            if metadata.get('payload_version') != artifact.payload_version:
                raise ValueError('payload version differs from manifest')
            events: list[dict] = []
            for event in metadata['events']:
                item = deepcopy(event)
                rgb_key = item.pop('rgb_key', None)
                depth_key = item.pop('depth_key', None)
                # A payload-version-1 capture has neither key, and reads back
                # with the scan explicitly absent rather than failing: every
                # event then answers "is there a scan" the same way, whether the
                # capture predates scan recording or simply missed one.
                scan_key = item.pop('scan_key', None)
                item['rgb'] = (
                    np.array(payload[rgb_key], dtype=np.uint8, copy=True)
                    if rgb_key is not None else None)
                item['depth_m'] = (
                    np.array(payload[depth_key], dtype=np.float32, copy=True)
                    if depth_key is not None else None)
                item['scan'] = (
                    {
                        **item['scan'],
                        'ranges': np.array(
                            payload[scan_key], dtype=np.float32, copy=True),
                    }
                    if scan_key is not None else None)
                _validate_sensor_event(item)
                events.append(item)
    except (OSError, KeyError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError(f'Cannot read sensor-capture payload {path}: {exc}') from exc
    return metadata['trial'], events


def load_mask_trial(
    artifact: ReplayArtifact,
    entry: dict,
) -> list[list[CachedMaskOutcome]]:
    if artifact.kind != MASK_CACHE_KIND:
        raise ValueError('load_mask_trial requires a mask-cache artifact.')
    path = artifact.root / entry['payload']
    try:
        with np.load(path, allow_pickle=False) as payload:
            metadata = json.loads(_scalar_string(payload['metadata_json']))
            if metadata.get('payload_version') != MASK_PAYLOAD_VERSION:
                raise ValueError('payload version differs from manifest')
            events: list[list[CachedMaskOutcome]] = []
            for serialized in metadata['events']:
                outcomes: list[CachedMaskOutcome] = []
                for item in serialized:
                    region_data = item.get('region')
                    region = None
                    if region_data is not None:
                        shape = tuple(int(value) for value in region_data['shape'])
                        size = int(np.prod(shape, dtype=np.int64))
                        packed = np.asarray(payload[region_data['data_key']], dtype=np.uint8)
                        unpacked = np.unpackbits(
                            packed, count=size, bitorder='little').astype(bool, copy=False)
                        region = MaskRegion(
                            data=np.array(unpacked.reshape(shape), dtype=bool, copy=True),
                            origin_u=int(region_data['origin_u']),
                            origin_v=int(region_data['origin_v']),
                            image_width=int(region_data['image_width']),
                            image_height=int(region_data['image_height']),
                            precision=MaskPrecision(str(region_data['precision'])),
                        )
                    outcomes.append(CachedMaskOutcome(
                        region=region,
                        status=str(item['status']),
                        duration_ms=(
                            None if item.get('duration_ms') is None
                            else float(item['duration_ms'])),
                    ))
                events.append(outcomes)
    except (OSError, KeyError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError(f'Cannot read mask-cache payload {path}: {exc}') from exc
    return events


def unavailable_estimators(evidence) -> dict[str, str]:
    """Which estimators this evidence cannot feed at all, and why.

    The test is about the ARTIFACT, not one event. An event whose scan or depth
    frame is missing is a miss the estimator reports for itself, and that frame
    stays in the tables saying so. This is the other case: the evidence holds no
    such channel anywhere, so every frame would carry the same miss and the row
    would describe the capture rather than the estimator.

    The reason travels with the drop because an absent row is indistinguishable
    from an estimator that simply scored nothing.

    One rule for both evidence formats, keyed on the version at which each began
    recording a scan. The legacy dataset and the sensor capture answer this
    differently today and will keep doing so; what must not differ is HOW they
    answer, because the GUI, the layered path, and the legacy path all read it.
    """

    if isinstance(evidence, ReplayArtifact):
        if evidence.kind != SENSOR_CAPTURE_KIND:
            raise ValueError(
                'Estimator availability is a question about capture evidence, '
                f'not a {evidence.kind!r} artifact.')
        format_name, version_label = 'sensor capture', 'payload version'
        version = evidence.payload_version
        first_scan_version = SENSOR_SCAN_PAYLOAD_VERSION
        unavailable = {'pointcloud': (
            'a sensor capture freezes aligned depth and the scan; the organized '
            'point cloud this estimator reads is never recorded')}
    else:
        # Imported inside, exactly as ``load_replay_input`` does it: the legacy
        # module is the heavy one and it reads this module, not the reverse.
        from ridgeback_autonomy.benchmarking.replay import REPLAY_SCAN_SCHEMA_VERSION
        format_name, version_label = 'legacy measurement dataset', 'schema version'
        version = int(evidence.manifest.get('schema_version', 0))
        first_scan_version = REPLAY_SCAN_SCHEMA_VERSION
        unavailable = {'pointcloud': (
            'a legacy measurement dataset freezes one depth ROI per detection; '
            'the organized point cloud this estimator reads is never recorded')}
    if first_scan_version is None:
        unavailable['polar_profiling'] = (
            f'no {version_label} of the {format_name} format records a LiDAR '
            'scan, so polar profiling has no beams to segment')
    elif version < first_scan_version:
        unavailable['polar_profiling'] = (
            f'this {format_name} is {version_label} {version}, written before '
            'the LiDAR scan was recorded')
    return unavailable


def select_replay_estimators(
    arguments: dict[str, str],
    unavailable: dict[str, str],
) -> tuple[tuple[str, ...], dict[str, str]]:
    """Split the requested estimators into the runnable ones and the refusals.

    Refusing outright when nothing survives is the point: an evaluation that
    quietly kept going would write a complete-looking run with every requested
    estimator missing from every table, which reads as "measured and found
    nothing" rather than "never ran".
    """

    requested = parse_estimators(arguments.get('estimators'))
    selected = tuple(
        estimator for estimator in requested if estimator not in unavailable)
    dropped = {
        estimator: unavailable[estimator]
        for estimator in requested if estimator in unavailable
    }
    if not selected:
        detail = '; '.join(
            f'{estimator}: {reason}' for estimator, reason in dropped.items())
        raise ValueError(
            f'This evidence can evaluate none of the requested estimators. {detail}')
    return selected, dropped


def trial_entry_map(artifact: ReplayArtifact) -> dict[str, dict]:
    return {str(entry['trial_id']): entry for entry in artifact.trial_entries}


def validate_cache_alignment(
    sensor: ReplayArtifact,
    caches: Iterable[ReplayArtifact],
) -> None:
    sensor_entries = trial_entry_map(sensor)
    for cache in caches:
        if cache.kind != MASK_CACHE_KIND:
            raise ValueError(f'Expected a mask cache, got {cache.kind!r}.')
        cache_entries = trial_entry_map(cache)
        if tuple(cache_entries) != tuple(sensor_entries):
            raise ValueError(
                f'Mask cache {cache.root} trial order differs from its sensor parent.')
        for trial_id, sensor_entry in sensor_entries.items():
            cache_entry = cache_entries[trial_id]
            for field in ('event_count', 'detection_count'):
                if int(cache_entry[field]) != int(sensor_entry[field]):
                    raise ValueError(
                        f'Mask cache {cache.root} {field} differs for trial {trial_id}.')
