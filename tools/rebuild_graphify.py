#!/usr/bin/env python3

from __future__ import annotations

import os
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = REPO_ROOT / "graphify-out"


def main() -> int:
    os.chdir(REPO_ROOT)

    from graphify.analyze import god_nodes, suggest_questions, surprising_connections
    from graphify.build import build_from_json
    from graphify.cluster import cluster, score_all
    from graphify.detect import detect, save_manifest
    from graphify.export import to_html, to_json
    from graphify.extract import extract
    from graphify.report import generate

    detection = detect(REPO_ROOT)
    code_files = [Path(path) for path in detection['files']['code']]
    if not code_files:
        print('No code files found for graph rebuild.', file=sys.stderr)
        return 1

    result = extract(code_files)
    graph = build_from_json(result)
    communities = cluster(graph)
    cohesion = score_all(graph, communities)
    labels = {community_id: f'Community {community_id}' for community_id in communities}
    gods = god_nodes(graph)
    surprises = surprising_connections(graph, communities)
    questions = suggest_questions(graph, communities, labels)

    OUT_DIR.mkdir(exist_ok=True)

    report = generate(
        graph,
        communities,
        cohesion,
        labels,
        gods,
        surprises,
        detection,
        {'input': 0, 'output': 0},
        str(REPO_ROOT),
        suggested_questions=questions,
    )

    (OUT_DIR / 'GRAPH_REPORT.md').write_text(report)
    to_json(graph, communities, str(OUT_DIR / 'graph.json'))
    to_html(graph, communities, str(OUT_DIR / 'graph.html'), community_labels=labels)
    save_manifest(detection['files'], str(OUT_DIR / 'manifest.json'))

    print(
        f'Rebuilt {OUT_DIR} with {graph.number_of_nodes()} nodes, '
        f'{graph.number_of_edges()} edges, {len(communities)} communities'
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
