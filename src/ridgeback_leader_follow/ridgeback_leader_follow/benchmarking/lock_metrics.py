"""Evaluation-only leader lock diagnostics; never imported by recognition nodes."""
import numpy as np
from scipy.optimize import linear_sum_assignment


def score_locks(predictions, truth_groups, annotations=None):
    states={};correct=wrong=locked=held=outside=outside_correct=0
    was_correct=False;ever_correct=False;loss_start=None;recoveries=[]
    for record in predictions:
        at=record['scenario_time'];state=record['state']
        states[state]=states.get(state,0)+1
        annotation=annotations.get(round(at,6)) if annotations is not None else None
        out_of_view=False
        if annotation is not None:
            leader=next((p for p in annotation['people'] if p['role']=='leader'),None)
            out_of_view=leader is not None and leader['box'] is None
            outside+=int(out_of_view)
        is_correct=False
        if state=='LOCKED':
            locked+=1
            people=truth_groups.get(round(at,6),[])
            target=next((p for p in record['people'] if p['id']==record['selected']),None)
            if target is not None and people:
                distances=np.array([np.hypot(target['x']-float(t['x']),target['y']-float(t['y']))
                                    for t in people])
                order=np.argsort(distances)
                unambiguous=len(order)==1 or distances[order[1]]-distances[order[0]] >= .1
                is_correct=(distances[order[0]] <= .75 and unambiguous
                            and people[order[0]]['role']=='leader')
            if is_correct:
                correct+=1
                held+=int(not target.get('appearance_available',False))
                outside_correct+=int(out_of_view)
            else:wrong+=1
        if was_correct and not is_correct:
            loss_start=at
        if is_correct and not was_correct and ever_correct and loss_start is not None:
            recoveries.append(at-loss_start);loss_start=None
        ever_correct=ever_correct or is_correct
        was_correct=is_correct
    return {'identity_label_policy':'selected target nearest truth within 0.75 m, separated by 0.1 m; unselected duplicate tracks cannot steal its label',
            'frames':len(predictions),'states':states,'locked_frames':locked,
            'correct_locked_frames':correct,'wrong_locked_frames':wrong,
            'correct_lock_fraction':correct/len(predictions) if predictions else None,
            'correct_locked_without_appearance_frames':held,
            'outside_camera_frames':outside if annotations is not None else None,
            'correct_locked_outside_camera_frames':outside_correct if annotations is not None else None,
            'outside_camera_lock_fraction':outside_correct/outside if outside else None,
            'recovery_seconds':recoveries,'unrecovered_loss':loss_start is not None,
            'passed_zero_wrong_lock':locked>0 and wrong==0}
