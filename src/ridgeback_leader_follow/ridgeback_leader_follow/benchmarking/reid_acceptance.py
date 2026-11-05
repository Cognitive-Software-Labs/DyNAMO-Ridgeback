"""Review F3 case evidence; missing camera-exit coverage is never a pass."""
import argparse
import csv
import json
from pathlib import Path
from .lock_metrics import score_locks
from .reid import CANDIDATES
from .dataset import SensorBag
from .stationary_anchor import align_records


def review(root,output):
    corpus=json.loads((root/'corpus.json').read_text())
    cases=corpus['results'];checks={}
    checks['sixteen_qualified_f3_cases']=(corpus.get('status')=='passed' and len(cases)==16
                                       and corpus.get('scene_profile')=='f3')
    checks['five_complete_candidate_comparisons']=all(
        (root/'comparison'/candidate/Path(case['run']).name/'score.json').exists()
        for candidate in CANDIDATES for case in cases)
    models={}
    for folder in sorted((root/'lock').glob('*')):
        if not folder.is_dir():continue
        results=[]
        for case in cases:
            run=Path(case['run']);key=run.name
            predictions=folder/key/'lock.jsonl'
            complete=folder/key/'score.json'
            if not predictions.exists() or not complete.exists():continue
            truth={}
            with (run/'ground_truth.csv').open() as stream:
                for row in csv.DictReader(stream):
                    truth.setdefault(round(float(row['scenario_time']),6),[]).append(row)
            annotations={round(row['scenario_time'],6):row for row in
                         (json.loads(line) for line in (root/'annotations'/(key+'.jsonl')).read_text().splitlines())}
            records=[json.loads(line) for line in predictions.read_text().splitlines()]
            if len(records)!=len(annotations):raise ValueError(f'Incomplete lock or annotation data: {key}')
            records=align_records(records,SensorBag(run),annotations)
            results.append({'case':key,**score_locks(records,truth,annotations)})
        models[folder.name]={'cases':results,'checks':{
            'complete_integrated_cases':len(results)==len(cases),
            'correct_leader_lock_in_every_case':len(results)==len(cases) and all(r['correct_locked_frames']>0 for r in results),
            'no_wrong_locks':bool(results) and all(r['wrong_locked_frames']==0 for r in results),
            'outside_camera_test_coverage':sum(r['outside_camera_frames'] or 0 for r in results)>0,
            'holds_correct_identity_outside_camera':sum(r['outside_camera_frames'] or 0 for r in results)>0 and
                all(r['outside_camera_frames']==r['correct_locked_outside_camera_frames'] for r in results)}}
    result={'status':'review_required','automatic_qualification':False,'checks':checks,'models':models,
            'policy':'Report all acceptance evidence; no automatic declaration that F3 is finished. No-wrong-lock is an additional conservative engineering check.',
            'scope':'simulation recognition; no robot movement or target hardware qualification'}
    output.write_text(json.dumps(result,indent=2)+'\n')
    return result


def review_selected(root, supplementary, ros_root, lock_profile, output):
    """Consolidate a named profile, including real ROS and camera-exit evidence."""
    cases=[]; checks={}; comparisons=[]
    for base, expected in ((root,16),(supplementary,2)):
        corpus=json.loads((base/'corpus.json').read_text())
        checks[base.name+'_qualified_corpus']=(corpus.get('status')=='passed' and
            corpus.get('scene_profile')=='f3' and len(corpus['results'])==expected)
        for case in corpus['results']:
            key=Path(case['run']).name
            offline=base/lock_profile/'osnet-ain'/key/'score.json'
            live=ros_root/key/'identity_score.json'
            publication=ros_root/key/'validation.json'
            cases.append({'case':key,'offline':json.loads(offline.read_text()) if offline.exists() else None,
                          'ros':json.loads(live.read_text()) if live.exists() else None,
                          'publication':json.loads(publication.read_text()) if publication.exists() else None})
    for candidate in CANDIDATES:
        reports=[root/'comparison'/candidate/Path(c['run']).name/'score.json'
                 for c in json.loads((root/'corpus.json').read_text())['results']]
        comparisons.append({'candidate':candidate,'complete_cases':sum(p.exists() for p in reports)})
    checks['five_complete_candidate_comparisons']=all(c['complete_cases']==16 for c in comparisons)
    for kind in ('offline','ros'):
        reports=[c[kind] for c in cases if c[kind] is not None]
        checks[kind+'_complete']=len(reports)==18
        checks[kind+'_correct_lock_every_case']=len(reports)==18 and all(r['correct_locked_frames']>0 for r in reports)
        checks[kind+'_zero_wrong_locks']=len(reports)==18 and all(r['wrong_locked_frames']==0 for r in reports)
        outside=sum(r['outside_camera_frames'] or 0 for r in reports)
        held=sum(r['correct_locked_outside_camera_frames'] or 0 for r in reports)
        checks[kind+'_camera_exit_covered']=outside>0
        checks[kind+'_all_camera_exit_observations_held']=outside>0 and held==outside
    checks['ros_feature_publication']=all(c['publication'] is not None and
        c['publication']['status']=='passed' and c['publication']['appearance_dimensions']==[512] for c in cases)
    result={'status':'evidence_checks_passed' if all(checks.values()) else 'incomplete_or_failed',
            'automatic_qualification':False,'checks':checks,'comparisons':comparisons,'cases':cases,
            'lock_profile':lock_profile,'ros_root':str(ros_root),
            'policy':'Review coverage, recovery and per-scenario accuracy before declaring F3 complete. Missing evidence never passes.',
            'scope':'stationary simulated observer; no driving or target-hardware timing claim'}
    output.write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--supplementary',type=Path)
    parser.add_argument('--ros-root',type=Path)
    parser.add_argument('--lock-profile',default='selected-final')
    args=parser.parse_args()
    if bool(args.supplementary) != bool(args.ros_root):parser.error('Supply both --supplementary and --ros-root')
    if args.ros_root:review_selected(args.root,args.supplementary,args.ros_root,args.lock_profile,args.output)
    else:review(args.root,args.output)


if __name__ == '__main__':main()
