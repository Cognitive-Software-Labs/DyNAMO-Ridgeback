#!/usr/bin/env python3
"""Generate evaluation annotations and box detector reports for a full corpus."""
import argparse
import json
from pathlib import Path
from ridgeback_leader_follow.benchmarking.dataset import SensorBag
from ridgeback_leader_follow.benchmarking.annotations import annotate
from ridgeback_leader_follow.benchmarking.image_metrics import detection_metrics


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('corpus','output'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--world-from-odom',nargs=4,type=float,required=True)
    a=p.parse_args();cases=json.loads((a.corpus/'corpus.json').read_text())['results']
    a.output.mkdir(parents=True,exist_ok=True)
    for case in cases:
        run=Path(case['run']);target=a.output/(run.name+'.jsonl')
        if target.exists():continue
        print('Annotations',run.name,flush=True)
        try:method=annotate(SensorBag(run),target,a.world_from_odom)
        except FileExistsError:continue
        target.with_suffix('.json').write_text(json.dumps(method,indent=2)+'\n')


if __name__ == '__main__':main()
