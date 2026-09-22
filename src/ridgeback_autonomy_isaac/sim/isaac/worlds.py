"""World-name resolution for the Isaac Sim runner.

Keeps the gz-era operator contract: `start_exploration.sh initial_test_world`
style short names keep working. A name resolves, in order, to:

1. a repo-converted stage: sim/isaac/usd/worlds/<name>.usda (via the
   installed package share, or the source tree when running uninstalled)
2. an entry in STOCK_WORLDS — Isaac asset-catalog environments used as
   new worlds (need the NVIDIA asset root; downloaded/cached by Kit)
3. a literal filesystem path to any .usd/.usda the operator supplies
"""
from __future__ import annotations

from pathlib import Path

# Measured robot geometry relative to base_link. World-relative heights must be
# derived from these values and the selected world's floor, never duplicated as
# absolute spawn or scan-plane constants.
BASE_LINK_FLOOR_CLEARANCE = 0.02617
LIDAR_BASE_Z = 0.2264

# Isaac stock environments served from the NVIDIA asset root (P7).
# Values are asset-root-relative; the runner prefixes the configured
# assets root (isaacsim.storage.native get_assets_root_path).
STOCK_WORLDS = {
    "warehouse": "/Isaac/Environments/Simple_Warehouse/warehouse.usd",
    "warehouse_full": "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
    "hospital": "/Isaac/Environments/Hospital/hospital.usd",
    "office": "/Isaac/Environments/Office/office.usd",
}

# Top surface of the floor at the supported spawn location, in world metres.
# Repo-converted stages use a 0.1 m slab centred at z=0; NVIDIA stock worlds
# place their walking surface at z=0.
WORLD_FLOOR_Z = {
    "empty": 0.05,
    "g1_distance_calibration": 0.05,
    "initial_test_world": 0.05,
    **{name: 0.0 for name in STOCK_WORLDS},
}


def _world_key(name_or_path: str | Path) -> str:
    value = str(name_or_path)
    path = Path(value)
    return path.stem if path.suffix.lower() in (".usd", ".usda", ".usdc", ".sdf") \
        else value


def floor_z_for_world(name_or_path: str | Path,
                      floor_z: float | None = None) -> float:
    """Return the selected world's floor top in metres.

    Named supported worlds own their floor height here. An arbitrary world
    path must provide ``floor_z`` explicitly because guessing from USD scene
    geometry is ambiguous at stepped or multi-level spawn locations.
    """
    if floor_z is not None:
        return float(floor_z)
    key = _world_key(name_or_path)
    try:
        return WORLD_FLOOR_Z[key]
    except KeyError as exc:
        raise ValueError(
            f"world {name_or_path!r} has no registered floor height; "
            "provide an explicit floor/spawn override") from exc


def spawn_z_for_world(name_or_path: str | Path,
                      floor_z: float | None = None) -> float:
    """base_link world height that seats the wheel mesh on the floor."""
    return floor_z_for_world(name_or_path, floor_z) + BASE_LINK_FLOOR_CLEARANCE


def lidar_plane_z_for_world(name_or_path: str | Path,
                            floor_z: float | None = None) -> float:
    """World-space UST-10LX scan plane derived from the same floor value."""
    return spawn_z_for_world(name_or_path, floor_z) + LIDAR_BASE_Z


def get_assets_root() -> str | None:
    """The configured NVIDIA asset root (S3/Nucleus), or None if unavailable.

    Stock worlds (STOCK_WORLDS) stream from here. Only importable inside a
    live Kit/SimulationApp process — returns None on a bare interpreter, so
    repo-local worlds keep resolving without it.
    """
    try:
        from isaacsim.storage.native import get_assets_root_path
        return get_assets_root_path()
    except Exception:
        return None


def _candidate_dirs() -> list[Path]:
    dirs = []
    here = Path(__file__).resolve().parent
    dirs.append(here / "usd" / "worlds")          # source tree / symlink-install
    try:
        from ament_index_python.packages import get_package_share_directory
        dirs.append(Path(get_package_share_directory("ridgeback_autonomy_isaac"))
                    / "sim" / "isaac" / "usd" / "worlds")
    except Exception:
        pass
    return dirs


def resolve_world(name_or_path: str, assets_root: str | None = None) -> str:
    """Return a loadable stage path/URL for a world name or explicit path."""
    for d in _candidate_dirs():
        for ext in (".usda", ".usd"):
            candidate = d / f"{name_or_path}{ext}"
            if candidate.exists():
                return str(candidate)

    if name_or_path in STOCK_WORLDS:
        if assets_root is None:
            raise ValueError(
                f"world {name_or_path!r} is an Isaac stock environment and "
                "needs the NVIDIA assets root (no repo-local USD found)")
        return assets_root.rstrip("/") + STOCK_WORLDS[name_or_path]

    p = Path(name_or_path)
    if p.suffix in (".usd", ".usda", ".usdc") and p.exists():
        return str(p)

    known = sorted({f.stem for d in _candidate_dirs() if d.is_dir()
                    for f in d.glob("*.usd*")} | set(STOCK_WORLDS))
    raise ValueError(f"unknown world {name_or_path!r}; known: {known}")
