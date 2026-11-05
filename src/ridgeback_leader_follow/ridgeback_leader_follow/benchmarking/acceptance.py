#!/usr/bin/env python3
"""Check the F2 simulation completion criteria against preserved evidence."""
import argparse
import json
import math
from pathlib import Path


def verify(root, output, gz_validation='f2-node-selected-gz-validation'):
    corpus=json.loads((root/'f2-corpus/corpus.json').read_text())
    matrix=json.loads((root/'f2-simulation-results-final.json').read_text())
    fusion=json.loads((root/'f2-fused-owlv2-lfe-ppn-selected/summary.json').read_text())
    assert corpus['sensor_status'] == 'passed' and len(corpus['results']) == 16
    assert matrix['status'] == 'complete' and matrix['candidate_runs'] == 320
    assert len(matrix['summaries']) == 20 and all(r['cases'] == 16 for r in matrix['summaries'])
    assert fusion['status'] == 'complete' and len(fusion['results']) == 16
    expected={Path(r['run']).name for r in corpus['results']}
    assert {r['run'] for r in fusion['results']} == expected
    assert all(r['prediction_frames'] == r['truth_frames'] == 400 for r in fusion['results'])
    nodes=[]
    for name in (gz_validation,'f2-node-selected-isaac-validation'):
        folder=root/name;validation=json.loads((folder/'validation.json').read_text())
        assert validation['status'] == 'passed' and validation['camera'] == 'owlv2' and validation['lidar'] == 'lfe-ppn'
        rows=[json.loads(line) for line in (folder/'people.jsonl').read_text().splitlines()]
        assert rows and any(r['people'] for r in rows)
        previous=-1.
        for row in rows:
            assert row['frame'] == 'odom' and row['sim_time'] > previous
            previous=row['sim_time'];ids=set()
            for person in row['people']:
                assert person['track_id'] not in ids;ids.add(person['track_id'])
                assert all(math.isfinite(person[k]) for k in ('x','y','vx','vy','confidence'))
                assert 0 <= person['confidence'] <= 1 and person['appearance'] == []
        nodes.append({'evidence':str(folder),**validation})
    result={'status':'complete','scope':'F2 simulation only','candidate_methods':20,'candidate_runs':320,
            'sensor_qualified_cases':16,'selected_profile':'OWLv2 + LFE-PPN + ByteTrack-assisted CV Kalman fusion',
            'selected_profile_reports':16,'ros_node_validations':nodes,
            'limitations':['dense crowds still have tracking errors','single human asset and one recording per configuration',
                           'workstation latency; robot/Thor performance deferred','F3 appearance and F4 closed loop remain open']}
    with output.open('x') as stream:json.dump(result,stream,indent=2);stream.write('\n')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--gz-validation',default='f2-node-selected-gz-validation')
    a=p.parse_args();print(json.dumps(verify(a.root,a.output,a.gz_validation),indent=2))


if __name__ == '__main__':main()
