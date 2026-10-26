"""Benchmark asset catalogs: logical scenario names to simulator resources.

Scenario YAML names a world, a target, and object models only logically. Each
simulator adapter installs a catalog that maps those names to its own native
world and spawnable resources; this module validates that mapping and resolves
it without knowing any simulator file format. Kept pure (no ROS) so a catalog
can be checked before anything is launched.

Catalog layout::

    schema: ridgeback_benchmark_assets/1
    worlds:
      <public world>:
        native_name: <simulator world name>
        resource: <path or URI>
        format: <resource format>
    targets:
      <logical target>: {resource: ..., format: ..., spawn_z_m: 0.0}
    objects:
      <logical object>: {resource: ..., format: ..., spawn_z_m: 0.0}

A resource is an explicit URI (``scheme://...``), kept verbatim, or a path
relative to the catalog file, resolved and required to exist. A target or
object ``format`` must be one of the spawn formats the simulator advertises.
``spawn_z_m`` is the optional world-frame height of the resource origin when
placed on the floor.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass
import hashlib
import math
import os
import re
from typing import Any

import yaml


CATALOG_SCHEMA = 'ridgeback_benchmark_assets/1'
DEFAULT_TARGET = 'g1'

_URI_PATTERN = re.compile(r'^[A-Za-z][A-Za-z0-9+.-]*://')
_SPAWNABLE_FIELDS = frozenset({'resource', 'format', 'spawn_z_m'})
_WORLD_FIELDS = frozenset({'native_name', 'resource', 'format'})
_TOP_LEVEL_FIELDS = frozenset({'schema', 'worlds', 'targets', 'objects'})


class AssetCatalogError(ValueError):
    """A catalog that is malformed or cannot serve the requested scenes."""


@dataclass(frozen=True)
class CatalogResource:
    """One resolved catalog resource.

    ``declared`` is the catalog's own spelling; ``uri`` is what a simulator
    receives: the explicit URI unchanged, or the resolved absolute path.
    """

    logical_name: str
    declared: str
    uri: str
    resource_format: str
    spawn_z_m: float = 0.0

    def identity(self) -> dict[str, Any]:
        return {
            'resource': self.declared,
            'uri': self.uri,
            'format': self.resource_format,
            'spawn_z_m': self.spawn_z_m,
        }


@dataclass(frozen=True)
class CatalogWorld:
    public_name: str
    native_name: str
    resource: CatalogResource

    def identity(self) -> dict[str, Any]:
        return {
            'public_name': self.public_name,
            'native_name': self.native_name,
            'resource': self.resource.declared,
            'uri': self.resource.uri,
            'format': self.resource.resource_format,
        }


@dataclass(frozen=True)
class AssetSelection:
    """The catalog entries one scenario run uses, validated together."""

    catalog: 'AssetCatalog'
    world: CatalogWorld
    target: CatalogResource
    objects: Mapping[str, CatalogResource]

    @property
    def spawn_formats(self) -> frozenset[str]:
        return frozenset(
            resource.resource_format
            for resource in (self.target, *self.objects.values())
        )

    def provenance(self) -> dict[str, Any]:
        return {
            'schema': self.catalog.schema,
            'path': self.catalog.path,
            'sha256': self.catalog.sha256,
            'world': self.world.identity(),
            'target': {'name': self.target.logical_name, **self.target.identity()},
            'objects': {
                name: resource.identity()
                for name, resource in sorted(self.objects.items())
            },
        }


@dataclass(frozen=True)
class AssetCatalog:
    path: str
    sha256: str
    schema: str
    worlds: Mapping[str, CatalogWorld]
    targets: Mapping[str, CatalogResource]
    objects: Mapping[str, CatalogResource]

    def select(
        self,
        world: str,
        target: str,
        object_models: Iterable[str],
    ) -> AssetSelection:
        """Resolve one run's assets, reporting every missing name at once."""

        models = sorted(set(object_models))
        missing = []
        if world not in self.worlds:
            missing.append(f'world "{world}"')
        if target not in self.targets:
            missing.append(f'target "{target}"')
        missing.extend(f'object "{model}"' for model in models if model not in self.objects)
        if missing:
            raise AssetCatalogError(
                f'Asset catalog {self.path} has no entry for {", ".join(missing)}.')
        return AssetSelection(
            catalog=self,
            world=self.worlds[world],
            target=self.targets[target],
            objects={model: self.objects[model] for model in models},
        )


def scene_object_models(scenes: Iterable[Any]) -> frozenset[str]:
    """Every logical object model the scenes place."""

    return frozenset(obj.model for scene in scenes for obj in scene.objects)


def _require_mapping(value: Any, where: str) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        raise AssetCatalogError(f'{where} must be a mapping, got {value!r}.')
    return value


def _require_fields(entry: Mapping[str, Any], allowed: frozenset[str], where: str) -> None:
    unknown = sorted(set(entry) - allowed)
    if unknown:
        raise AssetCatalogError(f'{where} has unknown fields: {", ".join(unknown)}.')


def _require_text(entry: Mapping[str, Any], field: str, where: str) -> str:
    value = entry.get(field)
    if not isinstance(value, str) or not value.strip():
        raise AssetCatalogError(f'{where}: "{field}" must be a non-empty string, got {value!r}.')
    return value.strip()


def _logical_name(name: Any, where: str) -> str:
    if not isinstance(name, str) or not name.strip() or name != name.strip():
        raise AssetCatalogError(f'{where}: logical names must be non-empty trimmed strings, '
                                f'got {name!r}.')
    return name


def _resolve_resource(
    logical_name: str,
    entry: Mapping[str, Any],
    catalog_dir: str,
    where: str,
) -> CatalogResource:
    declared = _require_text(entry, 'resource', where)
    resource_format = _require_text(entry, 'format', where).lower()
    spawn_z_m = entry.get('spawn_z_m', 0.0)
    if (isinstance(spawn_z_m, bool) or not isinstance(spawn_z_m, (int, float))
            or not math.isfinite(spawn_z_m)):
        raise AssetCatalogError(f'{where}: "spawn_z_m" must be a finite number, '
                                f'got {spawn_z_m!r}.')

    if _URI_PATTERN.match(declared):
        uri = declared
    elif os.path.isabs(declared):
        raise AssetCatalogError(
            f'{where}: resource "{declared}" must be relative to the catalog or an explicit URI.')
    else:
        uri = os.path.normpath(os.path.join(catalog_dir, declared))
        if not os.path.exists(uri):
            raise AssetCatalogError(f'{where}: resource "{declared}" not found at {uri}.')
    return CatalogResource(
        logical_name=logical_name,
        declared=declared,
        uri=uri,
        resource_format=resource_format,
        spawn_z_m=float(spawn_z_m),
    )


def _parse_spawnables(
    document: Mapping[str, Any],
    section: str,
    catalog_dir: str,
) -> dict[str, CatalogResource]:
    raw_entries = document[section] if section in document else {}
    entries = _require_mapping(raw_entries, f'"{section}"')
    resources = {}
    for name, raw in entries.items():
        where = f'{section}.{name}'
        logical_name = _logical_name(name, where)
        entry = _require_mapping(raw, where)
        _require_fields(entry, _SPAWNABLE_FIELDS, where)
        resources[logical_name] = _resolve_resource(logical_name, entry, catalog_dir, where)
    return resources


def parse_asset_catalog(
    document: Any,
    *,
    catalog_dir: str,
    path: str = '<memory>',
    sha256: str = '',
) -> AssetCatalog:
    """Validate a loaded catalog document; relative resources resolve under ``catalog_dir``."""

    document = _require_mapping(document, f'Asset catalog {path}')
    schema = document.get('schema')
    if schema != CATALOG_SCHEMA:
        raise AssetCatalogError(
            f'Asset catalog {path}: unsupported schema {schema!r}; expected "{CATALOG_SCHEMA}".')
    _require_fields(document, _TOP_LEVEL_FIELDS, f'Asset catalog {path}')

    worlds = {}
    raw_worlds = document['worlds'] if 'worlds' in document else {}
    world_entries = _require_mapping(raw_worlds, '"worlds"')
    for name, raw in world_entries.items():
        where = f'worlds.{name}'
        public_name = _logical_name(name, where)
        entry = _require_mapping(raw, where)
        _require_fields(entry, _WORLD_FIELDS, where)
        worlds[public_name] = CatalogWorld(
            public_name=public_name,
            native_name=_require_text(entry, 'native_name', where),
            resource=_resolve_resource(public_name, entry, catalog_dir, where),
        )
    if not worlds:
        raise AssetCatalogError(f'Asset catalog {path} must declare at least one world.')

    return AssetCatalog(
        path=path,
        sha256=sha256,
        schema=schema,
        worlds=worlds,
        targets=_parse_spawnables(document, 'targets', catalog_dir),
        objects=_parse_spawnables(document, 'objects', catalog_dir),
    )


def load_asset_catalog(path: str) -> AssetCatalog:
    """Load a catalog file, recording the SHA-256 of its exact bytes."""

    path = os.path.abspath(path)
    with open(path, 'rb') as handle:
        raw = handle.read()
    try:
        document = yaml.safe_load(raw)
    except yaml.YAMLError as exc:
        raise AssetCatalogError(f'Asset catalog {path} is not valid YAML: {exc}') from exc
    return parse_asset_catalog(
        document,
        catalog_dir=os.path.dirname(path),
        path=path,
        sha256=hashlib.sha256(raw).hexdigest(),
    )
