"""Offline recovery-input verification tests; no robot or service access."""

import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch


SPEC = importlib.util.spec_from_file_location(
    "verify_recovery", Path(__file__).with_name("verify_recovery.py")
)
verifier = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = verifier
SPEC.loader.exec_module(verifier)


class RecoveryVerifierTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / "deployment"
        self.root.mkdir()
        self.entries = []
        self.add_file("etc/config.conf", "enabled=true\n")

    def add_file(self, name, content, redactions=None):
        path = self.root / "files" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
        entry = {
            "source": "/" + name, "repo_path": "files/" + name,
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "mode": "0o644", "owner": "root", "group": "root",
            "source_symlink": None, "redacted_fields": redactions or [],
        }
        self.entries.append(entry)
        return path, entry

    def verify(self):
        (self.root / "manifest.json").write_text(json.dumps({"files": self.entries}))
        return verifier.verify(self.root)

    def assert_error(self, result, fragment):
        self.assertFalse(result.ok)
        self.assertTrue(any(fragment in error for error in result.errors), result.errors)

    def test_valid_small_fixture_has_no_fixed_file_count_or_host_dependency(self):
        self.add_file("usr/bin/example.py", "raise RuntimeError('must never execute')\n")
        self.add_file("etc/dds.xml", "<CycloneDDS><Domain id='any'/></CycloneDDS>\n")
        self.add_file("etc/web.yaml", 'node:\n  robot_password: "SET_LOCALLY_DURING_RECOVERY"\n',
                      ["robot_password"])
        self.add_file("usr/bin/example", "#!/usr/bin/env bash\nexit 9\n")
        result = self.verify()
        self.assertTrue(result.ok, result.errors)
        self.assertEqual(result.hashed, len(self.entries))
        self.assertEqual(result.syntax_checked, 4)
        self.assertEqual(result.redacted, 1)

    def test_tampered_hash(self):
        (self.root / self.entries[0]["repo_path"]).write_text("changed\n")
        self.assert_error(self.verify(), "SHA-256 mismatch")

    def test_missing_and_unlisted_files(self):
        (self.root / self.entries[0]["repo_path"]).unlink()
        (self.root / "files" / "extra").write_text("extra")
        result = self.verify()
        self.assert_error(result, "missing or nonregular")
        self.assert_error(result, "unlisted snapshot")

    def test_traversal_and_noncanonical_paths_are_rejected(self):
        for name in ("../outside", "/etc/passwd", "files/../../outside",
                     "files/./config", "files//config", "files/config\nsecret"):
            with self.subTest(name=name):
                self.entries[0]["repo_path"] = name
                self.assert_error(self.verify(), "unsafe repository path")

    def test_source_paths_must_be_canonical_absolute(self):
        for name in ("etc/config.conf", "/etc/../secret", "/etc//config.conf", "//etc/config", "/"):
            with self.subTest(name=name):
                self.entries[0]["source"] = name
                self.assert_error(self.verify(), "canonical absolute path")

    def test_duplicate_origins_and_paths(self):
        _, entry = self.add_file("etc/second.conf", "another\n")
        entry["source"] = self.entries[0]["source"]
        self.assert_error(self.verify(), "duplicate source origin")
        entry["source"] = "/etc/second.conf"
        entry["repo_path"] = self.entries[0]["repo_path"]
        self.assert_error(self.verify(), "duplicate repository path")

    def test_symlink_file_escape_is_never_read(self):
        outside = Path(self.temp.name) / "outside.py"
        outside.write_text("secret that is not valid Python")
        path, entry = self.add_file("usr/bin/example.py", "pass\n")
        path.unlink()
        path.symlink_to(outside)
        result = self.verify()
        self.assert_error(result, "snapshot symlink")
        self.assertEqual(result.syntax_checked, 0)
        self.assertEqual(result.hashed, 1)

    def test_symlink_directory_escape_and_files_root(self):
        outside = Path(self.temp.name) / "outside"
        outside.mkdir()
        (outside / "config").write_text("secret")
        (self.root / "files" / "escape").symlink_to(outside, target_is_directory=True)
        self.entries[0]["repo_path"] = "files/escape/config"
        result = self.verify()
        self.assert_error(result, "snapshot symlink")
        self.assert_error(result, "inventory directory symlink")
        import shutil
        shutil.rmtree(self.root / "files")
        (self.root / "files").symlink_to(outside, target_is_directory=True)
        self.assert_error(self.verify(), "missing directory or symlink")

    def test_redacted_secret_is_rejected_even_if_manifest_is_rehashed(self):
        path, entry = self.add_file("etc/web.yaml", "node:\n  robot_password: actual-secret\n",
                                    ["robot_password"])
        result = self.verify()
        self.assert_error(result, "not a placeholder")
        self.assertEqual(result.hashed, 2)
        self.assertNotIn("actual-secret", "\n".join(result.errors))
        path.write_text("node:\n  other_field: SET_LOCALLY_DURING_RECOVERY\n")
        entry["sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
        self.assert_error(self.verify(), "redacted field missing")

    def test_all_redacted_field_occurrences_are_checked(self):
        self.add_file("etc/web.yaml", "a:\n  robot_password: SET_LOCALLY_DURING_RECOVERY\n"
                      "b:\n  robot_password: actual-secret\n", ["robot_password"])
        self.assert_error(self.verify(), "not a placeholder")

    def test_invalid_metadata(self):
        for key, value, fragment in (
            ("owner", "root:root", "invalid owner"),
            ("group", "", "invalid group"),
            ("mode", "0o888", "invalid mode"),
            ("sha256", "not-a-hash", "invalid SHA-256"),
            ("redacted_fields", ["password", "password"], "invalid redacted_fields"),
            ("source_symlink", "relative/path", "source symlink metadata"),
        ):
            with self.subTest(key=key):
                original = self.entries[0][key]
                self.entries[0][key] = value
                self.assert_error(self.verify(), fragment)
                self.entries[0][key] = original

    def test_parser_errors_never_expose_source(self):
        self.add_file("usr/bin/broken.py", "if actual_secret invalid\n")
        self.add_file("etc/broken.xml", "<actual_secret>\n")
        self.add_file("etc/broken.yaml", "actual_secret: [\n")
        self.add_file("usr/bin/broken.sh", "#!/bin/bash\nif actual_secret\n")
        self.verify()
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            status = verifier.main(["--root", str(self.root)])
        self.assertEqual(status, 1)
        self.assertNotIn("actual_secret", output.getvalue())
        self.assertEqual(output.getvalue().count("invalid syntax"), 4)

    def test_missing_yaml_dependency_is_explicit_failure(self):
        self.add_file("etc/input.yaml", "key: value\n")
        with patch.object(verifier.importlib, "import_module", side_effect=ModuleNotFoundError):
            result = self.verify()
        self.assert_error(result, "PyYAML required")
        self.assertEqual(result.syntax_checked, 0)

    def test_duplicate_yaml_key_cannot_hide_a_secret(self):
        self.add_file("etc/web.yaml", "robot_password: actual-secret\n"
                      "robot_password: SET_LOCALLY_DURING_RECOVERY\n", ["robot_password"])
        self.assert_error(self.verify(), "duplicate YAML key in redacted template")

    def test_identical_nonredacted_duplicate_values_warn_without_blocking(self):
        self.add_file("etc/nav.yaml", "config:\n  set_initial_pose: true\n"
                      "  set_initial_pose: true\n")
        result = self.verify()
        self.assertTrue(result.ok, result.errors)
        self.assertEqual(result.syntax_checked, 1)
        self.assertEqual(len(result.warnings), 1)
        self.assertIn("identical-value duplicate YAML key at lines 2 and 3", result.warnings[0])
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            status = verifier.main(["--root", str(self.root)])
        self.assertEqual(status, 0)
        self.assertIn("WARN:", output.getvalue())
        self.assertIn("PASS", output.getvalue())

    def test_conflicting_nonredacted_duplicate_values_fail(self):
        self.add_file("etc/nav.yaml", "set_initial_pose: true\nset_initial_pose: false\n")
        self.assert_error(self.verify(), "conflicting duplicate YAML key")

    def test_equal_nested_duplicate_values_warn_but_changed_nested_values_fail(self):
        path, entry = self.add_file("etc/nav.yaml", "config: {pose: [1, 2]}\n"
                                    "config: {pose: [1, 2]}\n")
        result = self.verify()
        self.assertTrue(result.ok, result.errors)
        self.assertEqual(len(result.warnings), 1)
        path.write_text("config: {pose: [true, 2]}\nconfig: {pose: [1, 2]}\n")
        entry["sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
        self.assert_error(self.verify(), "conflicting duplicate YAML key")

    def test_identical_redacted_template_duplicates_fail(self):
        self.add_file("etc/web.yaml", "robot_password: SET_LOCALLY_DURING_RECOVERY\n"
                      "robot_password: SET_LOCALLY_DURING_RECOVERY\n", ["robot_password"])
        result = self.verify()
        self.assert_error(result, "duplicate YAML key in redacted template")
        self.assertEqual(result.warnings, [])

    def test_missing_bash_is_explicit_failure(self):
        self.add_file("usr/bin/script", "#!/bin/bash\nexit 1\n")
        with patch.object(verifier.shutil, "which", return_value=None):
            self.assert_error(self.verify(), "bash unavailable")

    def test_invalid_manifest_is_reported_without_values(self):
        (self.root / "manifest.json").write_text("secret is not JSON")
        result = verifier.verify(self.root)
        self.assert_error(result, "manifest:")
        self.assertNotIn("secret", str(result.errors))


if __name__ == "__main__":
    unittest.main()
