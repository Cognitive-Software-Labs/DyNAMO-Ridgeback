"""Simulators a target-distance benchmark can run on, and how each is selected.

The benchmark owns one persistent simulation environment per run or sweep, so
the backend is an environment property: the per-configuration layer receives it
only to find the same asset catalog and scene-control services, and to record
them. Everything else about a simulator stays in its adapter package; this
registry records just enough to select that package, find its catalog and
services, and validate the few launch settings only that simulator consumes.

Kept free of ROS imports (``ament_index_python`` is imported only when a
package share is actually looked up) so the sweep validator can use it, and so
selecting one backend never resolves another backend's package.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
import glob
import json
import math
import os
import re
from typing import Any
import xml.etree.ElementTree as ElementTree

from ridgeback_common.camera_profiles import (
    DEFAULT_DEPTH_FIDELITY,
    DEPTH_FIDELITY_CHOICES,
)

from ridgeback_autonomy.benchmarking.asset_catalog import (
    AssetCatalog,
    CatalogWorld,
    load_asset_catalog,
)


DEFAULT_BACKEND = 'gz'
# Relative to the adapter's installed share directory.
ASSET_CATALOG_RELATIVE_PATH = os.path.join('sim', 'benchmark_assets.yaml')

# Backends the wider stack knows that cannot host this benchmark, with the
# reason an operator is given instead of a launch failure later.
UNSUPPORTED_BACKENDS = {
    'hardware': (
        'physical hardware has no scene-control services and no ground truth, so '
        'it cannot place benchmark targets; the target-distance benchmark runs '
        'only in simulation'
    ),
}

PackageShareLookup = Callable[[str], str]


@dataclass(frozen=True)
class BackendSetting:
    """One environment launch argument only a single backend consumes.

    ``choices`` lists the accepted spellings; ``non_negative_float`` accepts a
    finite number at or above zero. Values are compared and recorded only in
    their normalized form, so ``rtf: 1`` and ``rtf: 1.0`` are one environment.
    """

    default: str
    description: str
    choices: tuple[str, ...] | None = None
    non_negative_float: bool = False

    def normalize(self, value: Any, *, owner: str, name: str) -> str:
        text = str(value).strip()
        if self.non_negative_float:
            try:
                number = float(text)
            except ValueError:
                number = math.nan
            if not math.isfinite(number) or number < 0.0:
                raise ValueError(
                    f'{owner}: field "{name}" must be a non-negative number, got {value!r}.')
            return repr(number)
        normalized = text.lower() if self.choices == _BOOLEAN else text
        if self.choices is not None and normalized not in self.choices:
            raise ValueError(
                f'{owner}: field "{name}" must be one of {", ".join(self.choices)}, '
                f'got {value!r}.')
        return normalized


_BOOLEAN = ('true', 'false')


@dataclass(frozen=True)
class BenchmarkBackend:
    name: str
    adapter_package: str
    # The provider namespace the adapter serves the standard scene-control
    # services under.
    scene_control_service_namespace: str
    settings: Mapping[str, BackendSetting] = field(default_factory=dict)

    def default_settings(self) -> dict[str, str]:
        return {name: setting.default for name, setting in self.settings.items()}


BENCHMARK_BACKENDS: Mapping[str, BenchmarkBackend] = {
    'gz': BenchmarkBackend(
        name='gz',
        adapter_package='ridgeback_autonomy_gz',
        scene_control_service_namespace='/gzserver',
        settings={
            'gz_gui': BackendSetting(
                default='true', choices=_BOOLEAN,
                description='Launch the Gazebo GUI window'),
            'headless_rendering': BackendSetting(
                default='false', choices=_BOOLEAN,
                description='Optional server-only EGL sensor rendering without an X display'),
        },
    ),
    'isaac': BenchmarkBackend(
        name='isaac',
        adapter_package='ridgeback_autonomy_isaac',
        scene_control_service_namespace='/isaac_sim',
        settings={
            'headless': BackendSetting(
                default='true', choices=_BOOLEAN,
                description='Run Isaac Sim without its viewport window'),
            'rtf': BackendSetting(
                default='1.0', non_negative_float=True,
                description='Isaac Sim real-time factor; 0 is unthrottled'),
            'sim_mode': BackendSetting(
                default='realtime', choices=('realtime', 'deterministic'),
                description='Isaac Sim stepping: realtime or fixed-step deterministic'),
            'depth_fidelity': BackendSetting(
                default=DEFAULT_DEPTH_FIDELITY, choices=DEPTH_FIDELITY_CHOICES,
                description='Isaac Sim depth: ideal renderer depth or D455-like stereo'),
        },
    ),
}

# Every backend-only environment setting, whichever backend consumes it.
BACKEND_SETTING_NAMES = frozenset(
    name for backend in BENCHMARK_BACKENDS.values() for name in backend.settings)


def benchmark_backend(name: Any) -> BenchmarkBackend:
    """The registered simulation backend called ``name``, or a clear refusal."""

    text = str(name).strip()
    if text in BENCHMARK_BACKENDS:
        return BENCHMARK_BACKENDS[text]
    supported = ', '.join(sorted(BENCHMARK_BACKENDS))
    if text in UNSUPPORTED_BACKENDS:
        raise ValueError(
            f'backend "{text}" cannot run the target-distance benchmark: '
            f'{UNSUPPORTED_BACKENDS[text]}. Use one of: {supported}.')
    raise ValueError(
        f'Unknown benchmark backend {name!r}; expected one of: {supported}.')


def normalize_backend_settings(
    backend: BenchmarkBackend,
    values: Mapping[str, Any],
    *,
    owner: str = 'Benchmark environment',
) -> dict[str, str]:
    """The backend's settings with defaults filled in, in normalized form.

    A setting that belongs to a different backend is refused rather than
    ignored: a sweep that asks for Isaac depth fidelity on Gazebo would
    otherwise record a mode the simulator never ran.
    """

    for name in sorted(set(values) & BACKEND_SETTING_NAMES):
        if name not in backend.settings:
            owners = ', '.join(sorted(
                other.name for other in BENCHMARK_BACKENDS.values()
                if name in other.settings))
            raise ValueError(
                f'{owner}: field "{name}" applies only to backend {owners}, '
                f'not "{backend.name}".')
    return {
        name: setting.normalize(values.get(name, setting.default), owner=owner, name=name)
        for name, setting in backend.settings.items()
    }


def _default_package_share(package: str) -> str:
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(package)


def asset_catalog_path(
    backend: BenchmarkBackend,
    *,
    package_share: PackageShareLookup | None = None,
) -> str:
    """The selected adapter's installed catalog; no other adapter is looked up."""

    lookup = package_share or _default_package_share
    return os.path.join(lookup(backend.adapter_package), ASSET_CATALOG_RELATIVE_PATH)


@dataclass(frozen=True)
class BenchmarkEnvironment:
    """One resolved persistent benchmark environment."""

    backend: BenchmarkBackend
    public_world: str
    world: CatalogWorld
    catalog: AssetCatalog
    settings: Mapping[str, str]

    @property
    def native_world(self) -> str:
        return self.world.native_name

    def signature(self) -> dict[str, Any]:
        """The exact values a resumed sweep must share with the one it resumes."""

        return {
            'backend': self.backend.name,
            'public_world': self.public_world,
            'native_world': self.native_world,
            'asset_catalog_schema': self.catalog.schema,
            'asset_catalog_sha256': self.catalog.sha256,
            'settings': dict(self.settings),
        }

    def provenance(self) -> dict[str, Any]:
        return {
            **self.signature(),
            'adapter_package': self.backend.adapter_package,
            'scene_control_service_namespace': self.backend.scene_control_service_namespace,
            'asset_catalog_path': self.catalog.path,
            'world_resource': self.world.resource.uri,
        }


def resolve_benchmark_environment(
    backend_name: Any,
    public_world: str,
    settings: Mapping[str, Any] | None = None,
    *,
    package_share: PackageShareLookup | None = None,
    owner: str = 'Benchmark environment',
) -> BenchmarkEnvironment:
    """Validate a backend, its settings, and its catalog's public world.

    The scenario-facing world name stays public; the adapter receives the
    catalog's ``native_name`` for it, so an operator never spells a simulator's
    own world file in a benchmark command or YAML.
    """

    backend = benchmark_backend(backend_name)
    normalized = normalize_backend_settings(backend, settings or {}, owner=owner)
    catalog = load_asset_catalog(asset_catalog_path(backend, package_share=package_share))
    world_name = str(public_world).strip()
    if world_name not in catalog.worlds:
        raise ValueError(
            f'{owner}: backend "{backend.name}" has no benchmark world "{world_name}" '
            f'in {catalog.path}; available: {", ".join(sorted(catalog.worlds))}.')
    return BenchmarkEnvironment(
        backend=backend,
        public_world=world_name,
        world=catalog.worlds[world_name],
        catalog=catalog,
        settings=normalized,
    )


def encode_backend_settings(settings: Mapping[str, str]) -> str:
    """Settings as one stable text value for a ROS string parameter."""

    return json.dumps(dict(settings), sort_keys=True, separators=(',', ':'))


def decode_backend_settings(text: str) -> dict[str, str]:
    if not str(text).strip():
        return {}
    document = json.loads(text)
    if not isinstance(document, dict):
        raise ValueError(f'Backend settings must encode a mapping, got {text!r}.')
    return {str(key): str(value) for key, value in document.items()}


def _package_version(share_directory: str) -> str | None:
    try:
        root = ElementTree.parse(os.path.join(share_directory, 'package.xml')).getroot()
    except (OSError, ElementTree.ParseError):
        return None
    version = root.findtext('version')
    return version.strip() if version else None


def _gazebo_version(lookup: PackageShareLookup) -> dict[str, str | None]:
    # The ROS vendor package versions itself 0.0.x; the Gazebo release it
    # vendors is stated in its description.
    try:
        share = lookup('gz_sim_vendor')
        root = ElementTree.parse(os.path.join(share, 'package.xml')).getroot()
    except (LookupError, OSError, ElementTree.ParseError, ValueError):
        return {'name': 'gz-sim', 'version': None}
    match = re.search(
        r'Vendor package for:\s*(\S+)\s+(\S+)', root.findtext('description') or '')
    if match is None:
        return {'name': 'gz-sim', 'version': None}
    return {'name': match.group(1), 'version': match.group(2)}


def _isaac_version(_lookup: PackageShareLookup) -> dict[str, str | None]:
    # The Isaac adapter starts its runner with ISAAC_PYTHON, or the workspace's
    # isaac_venv relative to the launch directory; read the installed
    # distribution of that same interpreter.
    interpreter = os.environ.get(
        'ISAAC_PYTHON', os.path.join(os.getcwd(), 'isaac_venv', 'bin', 'python3'))
    venv = os.path.dirname(os.path.dirname(interpreter))
    versions = sorted({
        os.path.basename(path)[len('isaacsim-'):-len('.dist-info')]
        for path in glob.glob(os.path.join(
            venv, 'lib', 'python3*', 'site-packages', 'isaacsim-*.dist-info'))
    })
    return {
        'name': 'isaacsim',
        'version': versions[-1] if len(versions) == 1 else None,
        'interpreter': interpreter,
    }


_SIMULATOR_VERSION_PROBES: Mapping[str, Callable[[PackageShareLookup], dict]] = {
    'gz': _gazebo_version,
    'isaac': _isaac_version,
}


def simulator_provenance(
    backend: BenchmarkBackend,
    *,
    package_share: PackageShareLookup | None = None,
) -> dict[str, Any]:
    """Adapter package and simulator release, best effort: unknown is ``None``."""

    lookup = package_share or _default_package_share
    try:
        adapter_version = _package_version(lookup(backend.adapter_package))
    except (LookupError, ValueError):
        adapter_version = None
    return {
        'adapter_package': backend.adapter_package,
        'adapter_version': adapter_version,
        'simulator': _SIMULATOR_VERSION_PROBES[backend.name](lookup),
    }
