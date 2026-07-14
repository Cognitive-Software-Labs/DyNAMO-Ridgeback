"""World-name resolution for the Isaac Sim runner.

Keeps the gz-era operator contract: `start_exploration.sh mock_hospital`
style short names keep working. A name resolves, in order, to:

1. a repo-converted stage: sim/isaac/usd/worlds/<name>.usda (via the
   installed package share, or the source tree when running uninstalled)
2. an entry in STOCK_WORLDS — Isaac asset-catalog environments used as
   new worlds (need the NVIDIA asset root; downloaded/cached by Kit)
3. a literal filesystem path to any .usd/.usda the operator supplies
"""
from __future__ import annotations

from pathlib import Path

# Isaac stock environments served from the NVIDIA asset root (P7).
# Values are asset-root-relative; the runner prefixes the configured
# assets root (isaacsim.storage.native get_assets_root_path).
STOCK_WORLDS = {
    "warehouse": "/Isaac/Environments/Simple_Warehouse/warehouse.usd",
    "warehouse_full": "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
    "hospital": "/Isaac/Environments/Hospital/hospital.usd",
    "office": "/Isaac/Environments/Office/office.usd",
}


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
        dirs.append(Path(get_package_share_directory("ridgeback_autonomy"))
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
