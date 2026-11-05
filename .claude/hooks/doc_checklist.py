#!/usr/bin/env python3
"""PreToolUse hook: show the documentation writing checklist before the first
Markdown edit of a session. The checklist itself is owned by
docs/project/documentation.md and read from there, never copied here."""
import json
from pathlib import Path
import re
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SKIP = ('graphify-out/', 'build/', 'install/', 'artifacts/')

event = json.load(sys.stdin)
path = Path(event.get('tool_input', {}).get('file_path', ''))
try:
    relative = path.resolve().relative_to(ROOT).as_posix()
except ValueError:
    sys.exit(0)
if path.suffix != '.md' or relative.startswith(SKIP):
    sys.exit(0)

seen = Path(tempfile.gettempdir()) / f'ridgeback-doc-checklist-{event.get("session_id", "none")}'
if seen.exists():
    sys.exit(0)
policy = (ROOT / 'docs/project/documentation.md').read_text(encoding='utf-8')
match = re.search(r'<!-- writing-checklist:start -->\n(.*?)<!-- writing-checklist:end -->', policy, re.S)
if not match:
    sys.exit(0)
seen.touch()
print(json.dumps({'hookSpecificOutput': {
    'hookEventName': 'PreToolUse',
    'additionalContext': 'Documentation writing checklist (docs/project/documentation.md):\n'
                         + match.group(1).strip()}}))
