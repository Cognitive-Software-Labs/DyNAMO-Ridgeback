#!/usr/bin/env python3
"""Read-only checks of captured recovery inputs; never inspect the live host.

Hashes describe repository snapshots, including redacted templates, not the
original installed contents. Owner/mode fields are checked as metadata only.
YAML syntax and redacted fields require PyYAML; no captured program is executed.
"""

from __future__ import annotations

import argparse
import ast
from dataclasses import dataclass, field
import hashlib
import importlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET


DEFAULT_ROOT = Path(__file__).resolve().parents[2] / "deployment" / "r100_0160"
REDACTION_PLACEHOLDER = "SET_LOCALLY_DURING_RECOVERY"
ACCOUNT = re.compile(r"(?:[a-z_][a-z0-9_-]*\$?|[0-9]+)\Z", re.ASCII)


@dataclass
class Verification:
    listed: int = 0
    hashed: int = 0
    syntax_checked: int = 0
    redacted: int = 0
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors


def _safe_path(value: object, *, absolute: bool) -> bool:
    if (not isinstance(value, str) or not value or "\\" in value
            or value.startswith("//")):
        return False
    if any(ord(char) < 32 or ord(char) == 127 for char in value):
        return False
    path = PurePosixPath(value)
    if path.is_absolute() != absolute or path.as_posix() != value:
        return False
    if any(part in (".", "..") for part in value.split("/")[1 if absolute else 0:]):
        return False
    if absolute:
        return len(path.parts) > 1
    return len(path.parts) > 1 and path.parts[0] == "files"


def _has_symlink(root: Path, relative: str) -> bool:
    current = root
    for part in PurePosixPath(relative).parts:
        current /= part
        if current.is_symlink():
            return True
    return False


class DuplicateYAMLKeyError(ValueError):
    """Safe diagnostic for a rejected duplicate, without keys or values."""


def _same_yaml_value(left: object, right: object) -> bool:
    """Compare parsed values without treating YAML true and integer 1 as equal."""
    if type(left) is not type(right):
        return False
    if isinstance(left, dict):
        return (left.keys() == right.keys()
                and all(_same_yaml_value(value, right[key]) for key, value in left.items()))
    if isinstance(left, list):
        return (len(left) == len(right)
                and all(_same_yaml_value(a, b) for a, b in zip(left, right)))
    return left == right


def _yaml_document(content: bytes, *, reject_duplicates: bool = False):
    """Parse YAML; tolerate identical duplicates only outside redacted templates."""
    yaml = importlib.import_module("yaml")
    warnings = []

    class UniqueLoader(yaml.SafeLoader):
        pass

    def mapping(loader, node, deep=False):
        loader.flatten_mapping(node)
        keys = {}
        values = {}
        for key_node, value_node in node.value:
            key = loader.construct_object(key_node, deep=True)
            value = loader.construct_object(value_node, deep=True)
            if key in keys:
                if reject_duplicates:
                    raise DuplicateYAMLKeyError("duplicate YAML key in redacted template")
                if not _same_yaml_value(values[key], value):
                    raise DuplicateYAMLKeyError("conflicting duplicate YAML key")
                warnings.append("identical-value duplicate YAML key "
                                f"at lines {keys[key]} and {key_node.start_mark.line + 1}")
            keys[key] = key_node.start_mark.line + 1
            values[key] = value
        return values

    UniqueLoader.add_constructor(
        yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, mapping
    )
    return yaml.load(content, Loader=UniqueLoader), warnings


def _field_values(document: object, name: str) -> list[object]:
    values = []
    pending = [document]
    visited = set()
    while pending:
        node = pending.pop()
        if not isinstance(node, (dict, list)) or id(node) in visited:
            continue
        visited.add(id(node))  # YAML aliases may form cycles.
        if isinstance(node, dict):
            values.extend(value for key, value in node.items() if key == name)
            pending.extend(node.values())
        else:
            pending.extend(node)
    return values


def _check_content(content: bytes, relative: str, redactions: list[str],
                   result: Verification, label: str) -> None:
    suffix = PurePosixPath(relative).suffix.lower()
    first_line = content.split(b"\n", 1)[0]
    shebang = first_line.startswith(b"#!")
    python = suffix == ".py" or (shebang and b"python" in first_line)
    shell = suffix == ".sh" or (
        shebang and re.search(rb"(?:/|\s)(?:ba|da|k|z)?sh(?:\s|$)", first_line)
    )
    try:
        if python:
            ast.parse(content)
            result.syntax_checked += 1
        elif shell:
            bash = shutil.which("bash")
            if not bash:
                result.errors.append(f"{label}: bash unavailable for shell syntax check")
                return
            # -n parses only; sanitize startup environment and suppress diagnostics
            # because parser errors can contain a credential-bearing source line.
            checked = subprocess.run(
                [bash, "--noprofile", "--norc", "-n"], input=content,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                env={"PATH": os.defpath, "LC_ALL": "C"}, timeout=10,
                check=False,
            )
            if checked.returncode:
                raise ValueError("invalid shell syntax")
            result.syntax_checked += 1
        elif suffix == ".xml":
            ET.fromstring(content)
            result.syntax_checked += 1
        elif suffix in (".yaml", ".yml"):
            try:
                document, warnings = _yaml_document(content, reject_duplicates=bool(redactions))
            except ModuleNotFoundError:
                result.errors.append(f"{label}: PyYAML required for YAML validation")
                return
            result.syntax_checked += 1
            result.warnings.extend(f"{label}: {warning}" for warning in warnings)
            for name in redactions:
                values = _field_values(document, name)
                if not values or any(value != REDACTION_PLACEHOLDER for value in values):
                    result.errors.append(f"{label}: redacted field missing or not a placeholder")
            if redactions:
                result.redacted += 1
        if redactions and suffix not in (".yaml", ".yml"):
            result.errors.append(f"{label}: unsupported format for redacted field validation")
    except DuplicateYAMLKeyError as error:
        result.errors.append(f"{label}: {error}")
    except Exception:
        # Do not echo parser messages, file contents, or manifest values.
        result.errors.append(f"{label}: invalid syntax or syntax check failed")


def verify(root: Path | str = DEFAULT_ROOT) -> Verification:
    """Validate manifest coverage and snapshot contents without host operations."""
    result = Verification()
    try:
        root = Path(root).resolve(strict=True)
        manifest_path = root / "manifest.json"
        if manifest_path.is_symlink() or not manifest_path.is_file():
            raise ValueError("manifest must be a regular file")
        manifest = json.loads(manifest_path.read_bytes())
        entries = manifest["files"]
        if not isinstance(entries, list) or not entries:
            raise ValueError("files must be a nonempty list")
    except (OSError, ValueError, TypeError, KeyError):
        result.errors.append("manifest: missing, unreadable, or invalid files list")
        return result

    result.listed = len(entries)
    paths: set[str] = set()
    origins: set[str] = set()
    for index, entry in enumerate(entries, 1):
        label = f"entry {index}"
        if not isinstance(entry, dict):
            result.errors.append(f"{label}: invalid metadata object")
            continue
        relative, origin = entry.get("repo_path"), entry.get("source")
        valid_path = _safe_path(relative, absolute=False)
        if not valid_path:
            result.errors.append(f"{label}: unsafe repository path")
        elif relative in paths:
            result.errors.append(f"{label}: duplicate repository path")
        else:
            paths.add(relative)
        if not _safe_path(origin, absolute=True):
            result.errors.append(f"{label}: source must be a canonical absolute path")
        elif origin in origins:
            result.errors.append(f"{label}: duplicate source origin")
        else:
            origins.add(origin)
        target = entry.get("source_symlink")
        if target is not None and not _safe_path(target, absolute=True):
            result.errors.append(f"{label}: invalid source symlink metadata")
        mode = entry.get("mode")
        if not isinstance(mode, str) or not re.fullmatch(r"0o[0-7]{3,4}", mode):
            result.errors.append(f"{label}: invalid mode metadata")
        for key in ("owner", "group"):
            value = entry.get(key)
            if not isinstance(value, str) or not ACCOUNT.fullmatch(value):
                result.errors.append(f"{label}: invalid {key} metadata")
        digest = entry.get("sha256")
        if not isinstance(digest, str) or not re.fullmatch(r"[a-f0-9]{64}", digest):
            result.errors.append(f"{label}: invalid SHA-256 metadata")
        redactions = entry.get("redacted_fields")
        if (not isinstance(redactions, list)
                or any(not isinstance(name, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name)
                       for name in redactions)
                or len(set(redactions)) != len(redactions)):
            result.errors.append(f"{label}: invalid redacted_fields metadata")
            redactions = []
        if not valid_path:
            continue
        try:
            # Captures are regular copies even when their installed origin was
            # a symlink. Reject all snapshot symlinks, including parent dirs.
            if _has_symlink(root, relative):
                result.errors.append(f"{label}: snapshot symlink is not allowed")
                continue
            path = root / relative
            if not path.is_file():
                result.errors.append(f"{label}: missing or nonregular snapshot file")
                continue
            content = path.read_bytes()
        except OSError:
            result.errors.append(f"{label}: snapshot file unreadable")
            continue
        if hashlib.sha256(content).hexdigest() != digest:
            result.errors.append(f"{label}: SHA-256 mismatch")
        else:
            result.hashed += 1
        _check_content(content, relative, redactions, result, label)

    files_root = root / "files"
    if files_root.is_symlink() or not files_root.is_dir():
        result.errors.append("files: missing directory or symlink is not allowed")
        return result
    try:
        def walk_error(_error):
            result.errors.append("files: inventory directory unreadable")

        for directory, dirs, files in os.walk(files_root, followlinks=False, onerror=walk_error):
            for name in dirs[:]:
                if (Path(directory) / name).is_symlink():
                    result.errors.append("files: inventory directory symlink is not allowed")
                    dirs.remove(name)
            for name in files:
                path = Path(directory) / name
                if path.is_symlink():
                    result.errors.append("files: inventory file symlink is not allowed")
                if path.relative_to(root).as_posix() not in paths:
                    result.errors.append("files: unlisted snapshot file")
    except OSError:
        result.errors.append("files: inventory unreadable")
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT,
                        help="deployment directory containing manifest.json and files/")
    result = verify(parser.parse_args(argv).root)
    print(f"Recovery inputs: {result.listed} listed, {result.hashed} hashes matched, "
          f"{result.syntax_checked} syntax checked, {result.redacted} redacted templates checked")
    for error in result.errors:
        print(f"FAIL: {error}")
    for warning in result.warnings:
        print(f"WARN: {warning}")
    print("PASS" if result.ok else f"FAILED: {len(result.errors)} check(s)")
    return 0 if result.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
