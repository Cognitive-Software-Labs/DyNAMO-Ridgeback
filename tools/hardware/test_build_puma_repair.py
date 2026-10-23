"""A real enclosing Git repository must not silently suppress applied patches."""
import sys
import subprocess
import tempfile
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).parent))
from build_puma_repair import apply_patch_checked


class PatchApplicationTests(unittest.TestCase):
    def test_patch_applies_under_ignored_nested_tree_and_reapplication_fails(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name)
            subprocess.run(['git','init','-q',str(root)],check=True)
            (root/'.gitignore').write_text('artifacts/\n')
            tree=root/'artifacts/export';tree.mkdir(parents=True)
            target=tree/'file.txt';target.write_text('before\n')
            patch=root/'change.patch'
            patch.write_text('diff --git a/file.txt b/file.txt\n--- a/file.txt\n+++ b/file.txt\n@@ -1 +1 @@\n-before\n+after\n')
            old=subprocess.run(['git','apply',str(patch)],cwd=tree,capture_output=True,text=True)
            self.assertEqual(old.returncode,0)
            self.assertEqual(target.read_text(),'before\n')
            apply_patch_checked(tree,patch)
            self.assertEqual(target.read_text(),'after\n')
            with self.assertRaises(subprocess.CalledProcessError):apply_patch_checked(tree,patch)


if __name__=='__main__':unittest.main()
