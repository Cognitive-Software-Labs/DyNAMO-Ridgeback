"""Recompute evaluation-only lock metrics from preserved prediction records."""
import argparse
import csv
import json
from pathlib import Path
from .lock_metrics import score_locks
from .dataset import SensorBag
from .stationary_anchor import align_records


def rescore(root,corpus,annotations):
    cases={Path(case['run']).name:Path(case['run']) for case in json.loads(corpus.read_text())['results']}
    count=0
    for score in root.glob('*/*/score.json'):
        key=score.parent.name
        if key not in cases or not (score.parent/'lock.jsonl').exists():continue
        old=json.loads(score.read_text());truth={}
        with (cases[key]/'ground_truth.csv').open() as stream:
            for row in csv.DictReader(stream):truth.setdefault(round(float(row['scenario_time']),6),[]).append(row)
        visibility={round(row['scenario_time'],6):row for row in
                    (json.loads(line) for line in (annotations/(key+'.jsonl')).read_text().splitlines())}
        records=[json.loads(line) for line in (score.parent/'lock.jsonl').read_text().splitlines()]
        if len(records)!=len(visibility):raise ValueError('Incomplete prediction/annotation pairing')
        backup=score.with_name('score.before-selected-target-metric.json')
        if not backup.exists():backup.write_text(score.read_text())
        manifest=json.loads((cases[key]/"manifest.json").read_text())
        if manifest.get("scene_profile")=="f3":
            records=align_records(records,SensorBag(cases[key]),visibility)
            old["world_anchor"]="stationary base_link planar identity via recorded TF"
        old.update(score_locks(records,truth,visibility))
        score.write_text(json.dumps(old,indent=2)+'\n');count+=1
    return count


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('root','corpus','annotations'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();print('Rescored',rescore(a.root,a.corpus,a.annotations),'lock cases')


if __name__ == '__main__':main()
