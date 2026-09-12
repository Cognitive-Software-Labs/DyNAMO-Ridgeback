#!/usr/bin/env python3
"""Print the shared replay/live capability contract."""

from __future__ import annotations

import argparse
import json

import yaml

from ridgeback_autonomy.benchmarking.replay_profiles import describe_capabilities


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Describe benchmark replay profiles and axes.')
    parser.add_argument('--json', action='store_true', help='Emit JSON instead of YAML')
    args = parser.parse_args(argv)
    document = describe_capabilities()
    if args.json:
        print(json.dumps(document, indent=2, sort_keys=True))
    else:
        print(yaml.safe_dump(document, sort_keys=False), end='')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
