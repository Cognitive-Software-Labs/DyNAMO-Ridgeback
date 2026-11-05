from ridgeback_leader_follow.benchmarking.lock_metrics import score_locks


def record(at,state='LOCKED',x=1.,appearance=True):
    return {'scenario_time':at,'state':state,'selected':1,
            'people':[{'id':1,'x':x,'y':0.,'appearance_available':appearance}]}


def test_camera_exit_and_recovery_are_scored_separately_from_missing_features():
    rows=[record(0.),record(.05,appearance=False),record(.1,'LOST'),record(.2)]
    truth={t:[{'x':1.,'y':0.,'role':'leader'}] for t in (0.,.05,.1,.2)}
    annotations={t:{'people':[{'role':'leader','box':None if t==.05 else [0,0,10,10]}]}
                 for t in truth}
    result=score_locks(rows,truth,annotations)
    assert result['correct_locked_frames']==3
    assert result['correct_locked_without_appearance_frames']==1
    assert result['outside_camera_frames']==1
    assert result['outside_camera_lock_fraction']==1
    assert result['recovery_seconds']==[.1]
    assert not result['unrecovered_loss']


def test_unobserved_camera_exit_coverage_is_unknown_and_wrong_lock_is_counted():
    rows=[record(0.,x=4.)]
    result=score_locks(rows,{0.:[{'x':4.,'y':0.,'role':'bystander'}]})
    assert result['wrong_locked_frames']==1
    assert result['outside_camera_frames'] is None
    assert result['outside_camera_lock_fraction'] is None
    assert not result['passed_zero_wrong_lock']


def test_unselected_duplicate_cannot_steal_selected_leaders_truth_label():
    selected=record(0.,x=1.2)
    selected['people'].append({'id':2,'x':1.,'y':0.,'appearance_available':True})
    result=score_locks([selected],{0.:[{'x':1.,'y':0.,'role':'leader'},
                                     {'x':4.,'y':0.,'role':'bystander'}]})
    assert result['correct_locked_frames']==1 and result['wrong_locked_frames']==0


def test_equidistant_selected_target_is_not_declared_correct():
    result=score_locks([record(0.,x=1.4)],{0.:[{'x':1.,'y':0.,'role':'leader'},
                                            {'x':1.8,'y':0.,'role':'bystander'}]})
    assert result['correct_locked_frames']==0 and result['wrong_locked_frames']==1
