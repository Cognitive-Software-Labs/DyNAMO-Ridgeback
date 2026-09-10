"""ROS-free capability contract for live and layered replay benchmarks.

This module is the one owner of profile IDs, stage ownership, public benchmark
axes, and question-to-profile recommendations.  CLI tools and the local
configurator consume :func:`describe_capabilities`; execution code consumes the
same dataclasses and validation helpers.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable

from ridgeback_autonomy.perception.target_localization.estimator_registry import (
    DEPTH_PATH_ESTIMATORS,
    PUBLIC_ESTIMATOR_ORDER,
    parse_estimators,
)
from ridgeback_autonomy.perception.target_localization.launch import (
    CONFIG_LAUNCH_ARGUMENT_NAMES,
    ENV_LAYER_CONFIG_KEYS,
    SHARED_BENCHMARK_ARGUMENT_DEFAULTS,
    SIMULATION_CAMERA_INPUTS,
)
from ridgeback_autonomy.perception.target_localization.core.ranging_defaults import (
    FRONT_PERCENTILE,
    INLIER_AHEAD_MARGIN_M,
    INLIER_BEHIND_MARGIN_M,
    MIN_VALID_SAMPLES,
)
from ridgeback_autonomy.perception.target_localization.core.segmentation import (
    PROMPT_PADDING_REL_DEFAULT,
    SEGMENTATION_MIN_PREDICTED_IOU_DEFAULT,
    SEGMENTATION_MODEL_DEFAULT,
)
from ridgeback_autonomy.perception.target_localization.core.isolation_2d import (
    ISOLATION_2D_RECIPES,
    NEAREST_MODE_BAND_M_DEFAULT,
)
from ridgeback_autonomy.perception.target_localization.core.isolation_3d import (
    ISOLATION_3D_NAMES,
)
from ridgeback_autonomy.perception.target_localization.core.polar_profiling import (
    MIN_VALID_RAYS_DEFAULT,
    RANGE_BAND_M_DEFAULT,
    RANGE_JUMP_M_DEFAULT,
)


PROFILE_MEASUREMENT = 'measurement'
PROFILE_MASK_OUTPUT = 'mask-output'
PROFILE_MASK_MODEL = 'mask-model'
PROFILE_LIVE_SYSTEM = 'live-system'

STAGE_SYSTEM = 'live-system'
STAGE_DETECTOR = 'detector'
STAGE_SENSOR = 'sensor-evidence'
STAGE_MASK = 'mask-production'
STAGE_MEASUREMENT = 'measurement'

ARTIFACT_LEGACY_MEASUREMENT = 'legacy-measurement'
ARTIFACT_SENSOR_CAPTURE = 'sensor-capture'
ARTIFACT_MASK_CACHE = 'mask-cache'


@dataclass(frozen=True)
class AxisSpec:
    name: str
    value_type: str
    default: Any
    stage: str
    label: str
    choices: tuple[str, ...] = ()
    minimum: float | int | None = None
    maximum: float | int | None = None
    estimators: tuple[str, ...] = ()
    description: str = ''

    def as_dict(self) -> dict[str, Any]:
        return {
            key: value
            for key, value in {
                'name': self.name,
                'type': self.value_type,
                'default': self.default,
                'stage': self.stage,
                'label': self.label,
                'choices': list(self.choices),
                'minimum': self.minimum,
                'maximum': self.maximum,
                'estimators': list(self.estimators),
                'description': self.description,
            }.items()
            if value not in (None, (), [], '')
        }


@dataclass(frozen=True)
class ProfileSpec:
    id: str
    label: str
    earliest_mutable_stage: str
    required_artifacts: tuple[str, ...]
    frozen_stages: tuple[str, ...]
    rerun_stages: tuple[str, ...]
    allowed_axes: frozenset[str]
    compatible_estimators: tuple[str, ...]
    compatible_depth_sources: tuple[str, ...]
    compatible_mask_gates: tuple[str, ...]
    supported_claims: tuple[str, ...]
    limitations: tuple[str, ...]
    executor: str
    default_overrides: tuple[tuple[str, Any], ...] = ()

    def as_dict(self) -> dict[str, Any]:
        defaults = {
            name: AXES[name].default
            for name in sorted(self.allowed_axes)
            if AXES[name].default != ''
        }
        defaults.update(dict(self.default_overrides))
        return {
            'id': self.id,
            'label': self.label,
            'earliest_mutable_stage': self.earliest_mutable_stage,
            'required_artifacts': list(self.required_artifacts),
            'frozen_stages': list(self.frozen_stages),
            'rerun_stages': list(self.rerun_stages),
            'allowed_axes': sorted(self.allowed_axes),
            'compatible_estimators': list(self.compatible_estimators),
            'compatible_depth_sources': list(self.compatible_depth_sources),
            'compatible_mask_gates': list(self.compatible_mask_gates),
            'supported_claims': list(self.supported_claims),
            'limitations': list(self.limitations),
            'executor': self.executor,
            'axis_defaults': defaults,
        }


class ProfileValidationError(ValueError):
    """A machine-readable validation failure shared by CLI and GUI."""

    def __init__(
        self,
        *,
        field: str,
        code: str,
        message: str,
        profile: str | None = None,
        suggested_profile: str | None = None,
    ) -> None:
        super().__init__(message)
        self.field = field
        self.code = code
        self.profile = profile
        self.suggested_profile = suggested_profile

    def as_dict(self) -> dict[str, Any]:
        return {
            key: value
            for key, value in {
                'field': self.field,
                'code': self.code,
                'message': str(self),
                'profile': self.profile,
                'suggested_profile': self.suggested_profile,
            }.items()
            if value is not None
        }


def _axis(
    name: str,
    value_type: str,
    default: Any,
    stage: str,
    label: str,
    **kwargs,
) -> AxisSpec:
    return AxisSpec(name, value_type, default, stage, label, **kwargs)


_DEPTH_ESTIMATORS = tuple(
    estimator for estimator in PUBLIC_ESTIMATOR_ORDER
    if estimator in DEPTH_PATH_ESTIMATORS)

# Measurement knobs retain the launch spelling so existing sweep YAML remains
# valid.  Materializer knobs are deliberately distinct from live-only node
# internals: they form the immutable producer signature of a mask cache.
AXES: dict[str, AxisSpec] = {
    'estimators': _axis(
        'estimators', 'estimator-list', 'projective_ranging', STAGE_MEASUREMENT,
        'Estimators', choices=_DEPTH_ESTIMATORS),
    'mask_depth_max_meters': _axis(
        'mask_depth_max_meters', 'number', 0.0, STAGE_MEASUREMENT,
        'Depth ceiling', minimum=0.0, estimators=_DEPTH_ESTIMATORS),
    'isolation_2d': _axis(
        'isolation_2d', 'string', 'nearest_mode_histogram', STAGE_MEASUREMENT,
        'Projective recipe', choices=tuple(sorted(ISOLATION_2D_RECIPES)),
        estimators=('projective_ranging',)),
    'isolation_2d_bin_width_m': _axis(
        'isolation_2d_bin_width_m', 'number', 0.05, STAGE_MEASUREMENT,
        '2D histogram bin width', minimum=0.0,
        estimators=('projective_ranging',)),
    'isolation_2d_band_m': _axis(
        'isolation_2d_band_m', 'number', NEAREST_MODE_BAND_M_DEFAULT,
        STAGE_MEASUREMENT, '2D near-surface band', minimum=0.0,
        estimators=('projective_ranging',)),
    'isolation_2d_min_bin_fraction': _axis(
        'isolation_2d_min_bin_fraction', 'number', 0.05, STAGE_MEASUREMENT,
        '2D minimum bin fraction', minimum=0.0, maximum=1.0,
        estimators=('projective_ranging',)),
    'min_valid_pixels': _axis(
        'min_valid_pixels', 'integer', MIN_VALID_SAMPLES, STAGE_MEASUREMENT,
        'Minimum valid samples', minimum=1, estimators=_DEPTH_ESTIMATORS),
    'isolation_3d': _axis(
        'isolation_3d', 'string', 'height_crop_nearest_mode_band',
        STAGE_MEASUREMENT, 'Euclidean recipe',
        choices=tuple(sorted(ISOLATION_3D_NAMES)),
        estimators=('euclidean_reconstruction',)),
    'isolation_3d_floor_margin_m': _axis(
        'isolation_3d_floor_margin_m', 'number', 0.05, STAGE_MEASUREMENT,
        'Floor margin', minimum=0.0,
        estimators=('euclidean_reconstruction',)),
    'isolation_3d_percentile': _axis(
        'isolation_3d_percentile', 'number', FRONT_PERCENTILE, STAGE_MEASUREMENT,
        'Front percentile', minimum=0.0, maximum=100.0,
        estimators=('euclidean_reconstruction',)),
    'isolation_3d_ahead_m': _axis(
        'isolation_3d_ahead_m', 'number', INLIER_AHEAD_MARGIN_M, STAGE_MEASUREMENT,
        '3D band ahead', minimum=0.0,
        estimators=('euclidean_reconstruction',)),
    'isolation_3d_behind_m': _axis(
        'isolation_3d_behind_m', 'number', INLIER_BEHIND_MARGIN_M, STAGE_MEASUREMENT,
        '3D band behind', minimum=0.0,
        estimators=('euclidean_reconstruction',)),
    'isolation_3d_bin_width_m': _axis(
        'isolation_3d_bin_width_m', 'number', 0.05, STAGE_MEASUREMENT,
        '3D histogram bin width', minimum=0.0,
        estimators=('euclidean_reconstruction',)),
    'isolation_3d_min_bin_fraction': _axis(
        'isolation_3d_min_bin_fraction', 'number', 0.05, STAGE_MEASUREMENT,
        '3D minimum bin fraction', minimum=0.0, maximum=1.0,
        estimators=('euclidean_reconstruction',)),
    # Polar profiling reads the LiDAR scan, which the frozen sensor capture does
    # not hold, so these three are measurement knobs that only the live profile
    # can actually move. The estimator restriction keeps them out of the offline
    # cards rather than offering a setting replay would silently ignore.
    'polar_range_jump_m': _axis(
        'polar_range_jump_m', 'number', RANGE_JUMP_M_DEFAULT, STAGE_MEASUREMENT,
        'Polar range jump', minimum=0.0, estimators=('polar_profiling',)),
    'polar_range_band_m': _axis(
        'polar_range_band_m', 'number', RANGE_BAND_M_DEFAULT, STAGE_MEASUREMENT,
        'Polar near band', minimum=0.0, estimators=('polar_profiling',)),
    'polar_min_valid_rays': _axis(
        'polar_min_valid_rays', 'integer', MIN_VALID_RAYS_DEFAULT,
        STAGE_MEASUREMENT, 'Minimum valid rays', minimum=1,
        estimators=('polar_profiling',)),
    'mask_producer': _axis(
        'mask_producer', 'choice', 'box', STAGE_MASK, 'Mask producer',
        choices=('box', 'slimsam')),
    'segmentation_model': _axis(
        'segmentation_model', 'string', SEGMENTATION_MODEL_DEFAULT, STAGE_MASK,
        'Segmentation checkpoint'),
    'segmentation_model_revision': _axis(
        'segmentation_model_revision', 'string', '', STAGE_MASK,
        'Checkpoint revision'),
    'segmentation_prompt_padding_rel': _axis(
        'segmentation_prompt_padding_rel', 'number', PROMPT_PADDING_REL_DEFAULT, STAGE_MASK,
        'Prompt padding', minimum=0.0),
    'segmentation_min_iou': _axis(
        'segmentation_min_iou', 'number', SEGMENTATION_MIN_PREDICTED_IOU_DEFAULT, STAGE_MASK,
        'Predicted-IoU floor', minimum=0.0, maximum=1.0),
    'segmentation_device': _axis(
        'segmentation_device', 'string', 'auto', STAGE_MASK,
        'Segmentation device'),
    'segmentation_dtype': _axis(
        'segmentation_dtype', 'string', 'auto', STAGE_MASK,
        'Segmentation dtype', choices=('auto', 'float32', 'float16', 'bfloat16')),
    'segmentation_preprocessing': _axis(
        'segmentation_preprocessing', 'string', 'transformers-default', STAGE_MASK,
        'Preprocessing contract'),
    'depth_source': _axis(
        'depth_source', 'choice', 'stereoscopic', STAGE_SENSOR, 'Depth source',
        choices=('stereoscopic', 'monocular')),
    'mask_gate': _axis(
        'mask_gate', 'choice', 'box', STAGE_MASK, 'Live mask gate',
        choices=('box', 'silhouette')),
    'detector_fps': _axis(
        'detector_fps', 'number', 5.0, STAGE_DETECTOR, 'Detector rate', minimum=0.0),
    'detector_debug': _axis(
        'detector_debug', 'boolean', False, STAGE_DETECTOR, 'Detector diagnostics'),
    'capture_batches': _axis(
        'capture_batches', 'integer', 5, STAGE_SENSOR, 'Raw batches per trial',
        minimum=1),
    'capture_drain_sec': _axis(
        'capture_drain_sec', 'number', 2.0, STAGE_SENSOR, 'Exact-evidence drain',
        minimum=0.0),
    'capture_timeout_sec': _axis(
        'capture_timeout_sec', 'number', 30.0, STAGE_SENSOR, 'Capture stall timeout',
        minimum=0.0),
    'capture_sec': _axis(
        'capture_sec', 'number', 10.0, STAGE_SYSTEM, 'Live capture window',
        minimum=0.0),
    'namespace': _axis(
        'namespace', 'string', SHARED_BENCHMARK_ARGUMENT_DEFAULTS['namespace'],
        STAGE_SYSTEM, 'Namespace'),
    'use_sim_time': _axis(
        'use_sim_time', 'boolean', True, STAGE_SYSTEM, 'Use simulation time'),
    'world': _axis(
        'world', 'string', SHARED_BENCHMARK_ARGUMENT_DEFAULTS['world'],
        STAGE_SYSTEM, 'World'),
    'color_topic': _axis(
        'color_topic', 'string', SIMULATION_CAMERA_INPUTS.color_image_topic,
        STAGE_SYSTEM, 'Color topic'),
    'estimate_viz': _axis(
        'estimate_viz', 'boolean', True, STAGE_SYSTEM, 'Estimate visualization'),
    'overlay': _axis(
        'overlay', 'boolean', True, STAGE_SYSTEM, 'Camera overlay'),
    'scenario': _axis(
        'scenario', 'string', '', STAGE_SYSTEM, 'Scenario YAML'),
    'repeats': _axis(
        'repeats', 'integer', 5, STAGE_SYSTEM, 'Repeats', minimum=1),
    'run_dir_name': _axis(
        'run_dir_name', 'string', '', STAGE_SYSTEM, 'Run directory name'),
    'settle_sec': _axis(
        'settle_sec', 'number', 2.0, STAGE_SYSTEM, 'Settle time', minimum=0.0),
    'replay_dataset_dir': _axis(
        'replay_dataset_dir', 'string', '', STAGE_SYSTEM,
        'Legacy measurement capture directory'),
    'sensor_capture_dir': _axis(
        'sensor_capture_dir', 'string', '', STAGE_SYSTEM,
        'Sensor capture directory'),
    'depth_topic': _axis(
        'depth_topic', 'string', SIMULATION_CAMERA_INPUTS.aligned_depth_topic,
        STAGE_SYSTEM, 'Depth topic'),
    'camera_info_topic': _axis(
        'camera_info_topic', 'string', SIMULATION_CAMERA_INPUTS.color_camera_info_topic,
        STAGE_SYSTEM, 'Camera info topic'),
    'depth_match_debug': _axis(
        'depth_match_debug', 'boolean', False, STAGE_SYSTEM, 'Depth match debug'),
    'record_video': _axis(
        'record_video', 'boolean', True, STAGE_SYSTEM, 'Record video'),
    'scan_topic': _axis(
        'scan_topic', 'string', 'sensors/lidar2d_0/scan', STAGE_SYSTEM,
        'Scan topic'),
    'pointcloud_topic': _axis(
        'pointcloud_topic', 'string',
        SIMULATION_CAMERA_INPUTS.organized_points_topic or '', STAGE_SYSTEM,
        'Pointcloud topic'),
    'base_frame': _axis(
        'base_frame', 'string', '<namespace>/robot/base_link', STAGE_SYSTEM,
        'Base frame'),
    'shutdown_on_complete': _axis(
        'shutdown_on_complete', 'boolean', False, STAGE_SYSTEM,
        'Shutdown on complete'),
}

# Public live arguments that are orchestration/display rather than experiment
# axes. They still belong to the shared validator, with live-system ownership.
_LIVE_ONLY_ARGUMENTS = {
    'namespace', 'use_sim_time', 'world', 'color_topic', 'estimate_viz',
    'overlay', 'scenario', 'repeats', 'output_dir', 'settle_sec',
    'replay_dataset_dir', 'sensor_capture_dir', 'depth_topic',
    'camera_info_topic', 'depth_match_debug', 'record_video', 'scan_topic',
    'pointcloud_topic', 'base_frame', 'run_dir_name', 'shutdown_on_complete',
    'setup_path',
}
for _name in sorted(_LIVE_ONLY_ARGUMENTS):
    AXES.setdefault(
        _name,
        _axis(_name, 'string', '', STAGE_SYSTEM, _name.replace('_', ' ').title()),
    )

MEASUREMENT_AXES = frozenset(
    name for name, spec in AXES.items() if spec.stage == STAGE_MEASUREMENT)
MASK_AXES = frozenset(
    name for name, spec in AXES.items()
    if spec.stage == STAGE_MASK and name != 'mask_gate')

# Polar profiling measures off the LiDAR scan, and a sensor capture freezes RGB,
# depth, intrinsics and transforms only. Moving a polar knob during replay would
# report a number the stored evidence cannot produce, so the offline profiles
# freeze these axes outright rather than accept a setting they would ignore.
# This is a missing-evidence limit, not the estimator-compatibility filter the
# GUI applies -- the measurement profile deliberately still admits the euclidean
# axes it cannot select, because that evidence *is* on disk.
SCAN_AXES = frozenset(
    name for name, spec in AXES.items()
    if spec.estimators == ('polar_profiling',))
OFFLINE_MEASUREMENT_AXES = MEASUREMENT_AXES - SCAN_AXES

_OFFLINE_LIMITS = (
    'Does not establish ROS scheduling, camera delivery, end-to-end latency, '
    'throughput, GPU contention, or simulator real-time factor.',
    'Production defaults still require confirmation in the live-system benchmark.',
)

PROFILES: dict[str, ProfileSpec] = {
    PROFILE_MEASUREMENT: ProfileSpec(
        id=PROFILE_MEASUREMENT,
        label='Tune measurement',
        earliest_mutable_stage=STAGE_MEASUREMENT,
        required_artifacts=(ARTIFACT_LEGACY_MEASUREMENT,),
        frozen_stages=(STAGE_DETECTOR, STAGE_SENSOR, STAGE_MASK),
        rerun_stages=(STAGE_MEASUREMENT,),
        allowed_axes=OFFLINE_MEASUREMENT_AXES,
        compatible_estimators=('projective_ranging',),
        compatible_depth_sources=('stereoscopic',),
        compatible_mask_gates=('box',),
        supported_claims=('accuracy', 'coverage', 'miss-reason', 'paired-results'),
        limitations=_OFFLINE_LIMITS,
        executor='legacy-measurement-replay',
    ),
    PROFILE_MASK_OUTPUT: ProfileSpec(
        id=PROFILE_MASK_OUTPUT,
        label='Compare frozen mask outputs',
        earliest_mutable_stage=STAGE_MEASUREMENT,
        required_artifacts=(ARTIFACT_SENSOR_CAPTURE, ARTIFACT_MASK_CACHE),
        frozen_stages=(STAGE_DETECTOR, STAGE_SENSOR, STAGE_MASK),
        rerun_stages=(STAGE_MEASUREMENT,),
        allowed_axes=OFFLINE_MEASUREMENT_AXES,
        compatible_estimators=_DEPTH_ESTIMATORS,
        compatible_depth_sources=('frozen',),
        compatible_mask_gates=('cache',),
        supported_claims=('accuracy', 'coverage', 'miss-reason', 'paired-results'),
        limitations=_OFFLINE_LIMITS,
        executor='mask-cache-measurement-replay',
    ),
    PROFILE_MASK_MODEL: ProfileSpec(
        id=PROFILE_MASK_MODEL,
        label='Try mask models or settings',
        earliest_mutable_stage=STAGE_MASK,
        required_artifacts=(ARTIFACT_SENSOR_CAPTURE,),
        frozen_stages=(STAGE_DETECTOR, STAGE_SENSOR),
        rerun_stages=(STAGE_MASK, STAGE_MEASUREMENT),
        allowed_axes=OFFLINE_MEASUREMENT_AXES | MASK_AXES,
        compatible_estimators=_DEPTH_ESTIMATORS,
        compatible_depth_sources=('frozen',),
        compatible_mask_gates=('box', 'silhouette'),
        supported_claims=(
            'accuracy', 'coverage', 'miss-reason', 'paired-results',
            'isolated-mask-production-time',
        ),
        limitations=_OFFLINE_LIMITS,
        executor='mask-materialization-and-measurement-replay',
    ),
    PROFILE_LIVE_SYSTEM: ProfileSpec(
        id=PROFILE_LIVE_SYSTEM,
        label='Measure the live system',
        earliest_mutable_stage=STAGE_SYSTEM,
        required_artifacts=(),
        frozen_stages=(),
        rerun_stages=(STAGE_SYSTEM, STAGE_DETECTOR, STAGE_SENSOR, STAGE_MASK, STAGE_MEASUREMENT),
        allowed_axes=frozenset(
            name for name in CONFIG_LAUNCH_ARGUMENT_NAMES | ENV_LAYER_CONFIG_KEYS
            if name in AXES),
        compatible_estimators=PUBLIC_ESTIMATOR_ORDER,
        compatible_depth_sources=('stereoscopic', 'monocular'),
        compatible_mask_gates=('box', 'silhouette'),
        supported_claims=(
            'accuracy', 'coverage', 'miss-reason', 'latency', 'throughput',
            'integration', 'gpu-contention', 'simulator-real-time-factor',
        ),
        limitations=(),
        executor='existing-live-benchmark',
        default_overrides=(('estimators', 'all'),),
    ),
}

QUESTIONS = (
    {
        'id': 'tune-measurement',
        'label': 'Tune measurement',
        'profile': PROFILE_MEASUREMENT,
    },
    {
        'id': 'compare-mask-outputs',
        'label': 'Compare box with current SlimSAM',
        'profile': PROFILE_MASK_OUTPUT,
    },
    {
        'id': 'try-mask-model',
        'label': 'Try another mask model or settings',
        'profile': PROFILE_MASK_MODEL,
    },
    {
        'id': 'measure-live-system',
        'label': 'Measure latency, throughput, or integration',
        'profile': PROFILE_LIVE_SYSTEM,
    },
)


def get_profile(profile_id: str) -> ProfileSpec:
    try:
        return PROFILES[str(profile_id)]
    except KeyError as exc:
        supported = ', '.join(PROFILES)
        raise ProfileValidationError(
            field='profile',
            code='unknown_profile',
            message=f'Unknown replay profile "{profile_id}". Expected one of: {supported}.',
        ) from exc


def suggested_profile_for_axis(axis: str) -> str | None:
    stage = AXES.get(axis).stage if axis in AXES else None
    # Scan axes are measurement-stage but need live LiDAR, so the stage alone
    # would suggest no profile at all; they only move under the live system.
    if axis in SCAN_AXES:
        return PROFILE_LIVE_SYSTEM
    if stage == STAGE_MASK:
        return PROFILE_MASK_MODEL
    if stage in (STAGE_SYSTEM, STAGE_DETECTOR, STAGE_SENSOR):
        return PROFILE_LIVE_SYSTEM
    return None


def validate_profile_axes(profile_id: str, axes: Iterable[str]) -> None:
    profile = get_profile(profile_id)
    for axis in sorted(set(axes)):
        if axis not in AXES:
            raise ProfileValidationError(
                field=axis,
                code='unknown_axis',
                profile=profile.id,
                message=f'Unknown benchmark parameter "{axis}".',
            )
        if axis not in profile.allowed_axes:
            suggested = suggested_profile_for_axis(axis)
            correction = (
                f'; choose profile "{suggested}" to vary it'
                if suggested is not None else '')
            raise ProfileValidationError(
                field=axis,
                code='upstream_stage_frozen',
                profile=profile.id,
                suggested_profile=suggested,
                message=(
                    f'{axis} is frozen by profile "{profile.id}"{correction}.'
                ),
            )


def validate_axis_values(values: dict[str, Any]) -> None:
    """Validate values against the same type/range contract the UI renders."""

    for name, raw in values.items():
        spec = AXES.get(name)
        if spec is None:
            raise ProfileValidationError(
                field=name, code='unknown_axis',
                message=f'Unknown benchmark parameter "{name}".')
        try:
            if spec.value_type == 'integer':
                if isinstance(raw, bool):
                    raise ValueError
                value = int(raw)
                if str(value) != str(raw).strip():
                    raise ValueError
            elif spec.value_type == 'number':
                if isinstance(raw, bool):
                    raise ValueError
                value = float(raw)
                if not math.isfinite(value):
                    raise ValueError
            elif spec.value_type == 'boolean':
                if isinstance(raw, bool):
                    value = raw
                elif str(raw).strip().lower() in ('true', 'false'):
                    value = str(raw).strip().lower() == 'true'
                else:
                    raise ValueError
            elif spec.value_type == 'estimator-list':
                selected = parse_estimators(str(raw))
                if any(estimator not in spec.choices for estimator in selected):
                    raise ValueError
                value = str(raw).strip()
            else:
                value = str(raw).strip()
        except (TypeError, ValueError) as exc:
            raise ProfileValidationError(
                field=name, code='invalid_type',
                message=f'Benchmark parameter "{name}" must be {spec.value_type}.',
            ) from exc
        if spec.choices and spec.value_type != 'estimator-list' and value not in spec.choices:
            raise ProfileValidationError(
                field=name, code='invalid_choice',
                message=(
                    f'Benchmark parameter "{name}" must be one of: '
                    f'{", ".join(spec.choices)}.'),
            )
        if spec.minimum is not None and value < spec.minimum:
            raise ProfileValidationError(
                field=name, code='out_of_range',
                message=f'Benchmark parameter "{name}" must be at least {spec.minimum}.')
        if spec.maximum is not None and value > spec.maximum:
            raise ProfileValidationError(
                field=name, code='out_of_range',
                message=f'Benchmark parameter "{name}" must be at most {spec.maximum}.')


def describe_capabilities() -> dict[str, Any]:
    """Stable JSON-compatible profile and parameter contract."""

    return {
        'contract_version': 1,
        'profiles': [profile.as_dict() for profile in PROFILES.values()],
        'axes': [AXES[name].as_dict() for name in sorted(AXES)],
        'questions': [dict(question) for question in QUESTIONS],
        'artifact_kinds': [
            ARTIFACT_LEGACY_MEASUREMENT,
            ARTIFACT_SENSOR_CAPTURE,
            ARTIFACT_MASK_CACHE,
        ],
    }
