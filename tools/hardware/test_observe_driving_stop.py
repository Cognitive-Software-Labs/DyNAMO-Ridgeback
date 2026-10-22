"""A physical test must remain unarmed on released/stale stops or held controls."""
import sys
import subprocess
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

sys.path.insert(0,str(Path(__file__).parent))
from observe_driving_stop import await_pressed_stop, controller_neutral, release_sample_safe, disconnect_due, held_forward_input, bounded_permit_window, forward_trial_command_safe, cancel_permit_on_input_loss, feedback_fault_due, connected_controller
from observe_driving_stop import permit_odometry_safe, held_direction_input, directional_trial_command_safe, DEADMAN_YAW_TOLERANCE, DIRECTIONS
from observe_driving_stop import changed_motion_axes, mapping_input_ready, deadman_released_with_forward, disconnected_stationary_tail, stop_action_evidence


def stop(at, asserted):
    return {'topic':'platform/emergency_stop','elapsed':at,'message':{'data':asserted}}


class ArmingTests(unittest.TestCase):
    def test_stop_action_evidence_requires_action_before_budget_and_continued_permits(self):
        def row(t, topic, message):
            return {'elapsed': t, 'topic': topic, 'message': message}
        rows = [row(.1, 'observer/test_permit_sent', {'data': True}),
                row(.45, 'platform/motors/cmd', {'velocity': [.2]*4}),
                row(.55, 'observer/test_permit_sent', {'data': True}),
                row(.58, 'platform/motors/cmd', {'velocity': [0.]*4}),
                row(.585, 'platform/cmd_vel_limited', {'twist': {
                    'linear': {'x': 0., 'y': 0.}, 'angular': {'z': 0.}}}),
                row(.59, 'platform/emergency_stop_mux', {'data': False})]
        self.assertTrue(stop_action_evidence(rows, 'deadman', .5)['qualified_command_chain'])
        self.assertFalse(stop_action_evidence(rows, 'deadman', None)['qualified_command_chain'])
        self.assertFalse(stop_action_evidence(rows, 'deadman', 1.05)['qualified_command_chain'])
        self.assertFalse(stop_action_evidence(rows[:2]+rows[3:], 'deadman', .5)['qualified_command_chain'])
        self.assertFalse(stop_action_evidence(rows[:3], 'deadman', .5)['qualified_command_chain'])
        self.assertFalse(stop_action_evidence(rows, 'e-stop', .5)['qualified_command_chain'])
        rows[-1]['message']['data'] = True
        self.assertTrue(stop_action_evidence(rows, 'e-stop', .5)['qualified_command_chain'])
        rows.append(row(.6, 'platform/motors/cmd', {'velocity': [.1]*4}))
        self.assertFalse(stop_action_evidence(rows, 'e-stop', .5)['qualified_command_chain'])

    def test_deadman_release_is_distinct_from_centering_or_diagonal_input(self):
        message={'buttons':[0]*15,'axes':[0.,.4,1.,0.,0.,1.]}
        self.assertTrue(deadman_released_with_forward(message))
        message['buttons'][4]=1
        self.assertFalse(deadman_released_with_forward(message))
        message['buttons'][4]=0;message['axes'][1]=0.
        self.assertFalse(deadman_released_with_forward(message))
        message['axes'][1]=.4;message['axes'][3]=.2
        self.assertFalse(deadman_released_with_forward(message))
        message['axes'][3]=0.;message['axes'][1]=float('nan')
        self.assertFalse(deadman_released_with_forward(message))

    def test_stop_action_requires_forward_bounded_permit(self):
        script=str(Path(__file__).with_name('observe_driving_stop.py'))
        for extras in (['--test','stationary','--stop-action','deadman'],
                       ['--test','interlock-permit','--direction','left','--stop-action','e-stop']):
            result=subprocess.run([sys.executable,script,*extras,'--run-dir','/tmp/not-used'],
                                  capture_output=True,text=True)
            self.assertEqual(result.returncode,2)
            self.assertIn('requires a forward interlock-permit',result.stderr)

    def test_disconnected_tail_needs_pressed_stop_and_fresh_zero_wheels(self):
        rows=[stop(9.,True),stop(9.5,True)]
        for i in range(20):
            t=9.6+i*.02
            rows.append({'elapsed':t,'topic':'platform/odom','message':{'twist':{'twist':{
                'linear':{'x':0.,'y':0.},'angular':{'z':0.}}}}})
        for i in range(10):
            t=9.8+i*.02
            rows.append({'elapsed':t,'topic':'platform/motors/cmd','message':{'velocity':[0.]*4}})
            rows.append({'elapsed':t,'topic':'platform/cmd_vel_limited','message':{
                'twist':{'linear':{'x':0.,'y':0.},'angular':{'z':0.}}}})
        self.assertTrue(disconnected_stationary_tail(rows,10.))
        rows[-1]['message']['twist']['linear']['x']=.01
        self.assertFalse(disconnected_stationary_tail(rows,10.))
        rows[-1]['message']['twist']['linear']['x']=0.
        rows[1]['message']['data']=False
        self.assertFalse(disconnected_stationary_tail(rows,10.))
        rows[1]['message']['data']=True
        self.assertFalse(disconnected_stationary_tail(rows,10.3))

    def test_mapping_accepts_sticks_only_with_both_enable_buttons_released(self):
        message={'buttons':[0]*15,'axes':[0.,0.,1.,.8,0.,1.]}
        self.assertTrue(mapping_input_ready(message))
        for button in(4,5):
            message['buttons'][button]=1
            self.assertFalse(mapping_input_ready(message))
            message['buttons'][button]=0
        self.assertTrue(mapping_input_ready(message))
        self.assertFalse(mapping_input_ready({'buttons':[]}))
    def test_mapping_ignores_resting_trigger_axes_and_identifies_real_motion_axes(self):
        message={'axes':[0.,0.,1.,0.,0.,1.]}
        self.assertEqual(changed_motion_axes(message),[])
        message['axes'][3]=.8
        self.assertEqual(changed_motion_axes(message),[3])
        message['axes'][0]=.9
        self.assertEqual(changed_motion_axes(message),[0,3])
        message['axes'][3]=float('nan')
        with self.assertRaises(ValueError):changed_motion_axes(message)
        with self.assertRaises(ValueError):changed_motion_axes({'axes':[]})

    def test_mapping_cannot_be_used_for_a_motion_trial_or_without_interlock(self):
        base=[sys.executable,str(Path(__file__).with_name('observe_driving_stop.py')),
              '--mapping-axis','3','--run-dir','/tmp/not-used']
        for extra in [[],['--test','interlock-permit','--test-interlock','--operator-confirmed']]:
            result=subprocess.run(base+extra,capture_output=True,text=True)
            self.assertEqual(result.returncode,2)
            self.assertIn('mapping-axis requires stationary',result.stderr)

    def test_each_installed_joystick_axis_selects_only_the_requested_direction(self):
        joystick_axes = {0: 1, 1: 3, 2: 0}
        for direction, (index, sign) in DIRECTIONS.items():
            m={'buttons':[0]*15,'axes':[0.]*6}
            m['buttons'][4]=1
            m['axes'][joystick_axes[index]] = sign*.2
            with self.subTest(direction=direction):
                for candidate in DIRECTIONS:
                    self.assertEqual(held_direction_input(m,candidate), candidate==direction)
                for axis in (0,1,3,4):
                    if axis==joystick_axes[index] or (index==0 and axis==4):continue
                    mixed={'buttons':m['buttons'][:],'axes':m['axes'][:]}
                    mixed['axes'][axis]=.2
                    self.assertFalse(held_direction_input(mixed,direction))
                m['buttons'][4]=0
                self.assertFalse(held_direction_input(m,direction))

    def test_all_axis_command_caps_and_odometry_limits_are_independent(self):
        for direction,(index,sign) in DIRECTIONS.items():
            v=[0.,0.,0.];v[index]=sign*(.1 if index==2 else .05)
            def twist(values):
                return {'linear':{'x':values[0],'y':values[1],'z':0.},
                        'angular':{'x':0.,'y':0.,'z':values[2]}}
            with self.subTest(direction=direction):
                self.assertTrue(directional_trial_command_safe(twist(v),direction))
                self.assertTrue(permit_odometry_safe(v,direction))
                wrong=[-x for x in v]
                self.assertFalse(directional_trial_command_safe(twist(wrong),direction))
                self.assertFalse(permit_odometry_safe(wrong,direction))
                over=v[:];over[index]*=1.01
                self.assertFalse(directional_trial_command_safe(twist(over),direction))
                for other in range(3):
                    if other==index:continue
                    mixed=v[:];mixed[other]=.011 if other==2 else .001
                    self.assertFalse(directional_trial_command_safe(twist(mixed),direction))
                for value in [float('nan'),float('inf')]:
                    bad=v[:];bad[index]=value
                    self.assertFalse(directional_trial_command_safe(twist(bad),direction))
                    self.assertFalse(permit_odometry_safe(bad,direction))

    def test_backward_input_requires_enable_negative_forward_axis_and_no_side_axis(self):
        m={'buttons':[0]*15,'axes':[0.,-.2,0.,0.,0.,0.]}
        self.assertFalse(held_forward_input(m,'backward'))
        m['buttons'][4]=1
        self.assertTrue(held_forward_input(m,'backward'))
        self.assertFalse(held_forward_input(m))
        m['axes'][1]=.2
        self.assertFalse(held_forward_input(m,'backward'))
        m['axes'][1]=-.2;m['axes'][0]=.2
        self.assertFalse(held_forward_input(m,'backward'))

    def test_backward_command_and_odometry_reverse_only_longitudinal_sign(self):
        t={'linear':{'x':-.05,'y':0.,'z':0.},'angular':{'x':0.,'y':0.,'z':.003}}
        self.assertTrue(forward_trial_command_safe(t,'backward'))
        self.assertFalse(forward_trial_command_safe(t))
        for speed in [.01,-.051,float('nan')]:
            t['linear']['x']=speed
            self.assertFalse(forward_trial_command_safe(t,'backward'))
        self.assertTrue(permit_odometry_safe([-.05,0,0],'backward'))
        for v in [[.02,0,0],[-.076,0,0],[-.05,.021,0],[-.05,0,.051],[float('nan'),0,0]]:
            self.assertFalse(permit_odometry_safe(v,'backward'))
        self.assertTrue(permit_odometry_safe([.05,0,0]))
        self.assertFalse(permit_odometry_safe([-.02,0,0]))

    def test_backward_cannot_change_disconnect_or_feedback_fault_directions(self):
        for test in ['interlock-disconnect','interlock-feedback-fault','stationary']:
            result=subprocess.run([sys.executable,str(Path(__file__).with_name('observe_driving_stop.py')),
                '--test',test,'--direction','backward','--run-dir','/tmp/not-used'],
                capture_output=True,text=True)
            self.assertEqual(result.returncode,2)
            self.assertIn('applies only to interlock-permit',result.stderr)

    def test_feedback_fault_requires_configured_live_controller_link(self):
        with TemporaryDirectory() as directory:
            config = Path(directory) / 'controllers.txt'
            config.write_text('AC:36:1B:89:A1:0A\n')
            calls = []
            def query(argv, **kwargs):
                calls.append(argv)
                return subprocess.CompletedProcess(argv, 0, 'Connected: yes\n', '')
            self.assertTrue(connected_controller(config, query))
            self.assertEqual(calls[0][-2:], ['info', 'AC:36:1B:89:A1:0A'])
            def disconnected(argv, **kwargs):
                return subprocess.CompletedProcess(argv, 0, 'Connected: no\n', '')
            self.assertFalse(connected_controller(config, disconnected))
            config.write_text('AC:36:1B:89:A1:0A\nAA:BB:CC:DD:EE:FF\n')
            self.assertFalse(connected_controller(config, query))

    def test_feedback_fault_requires_both_explicit_motion_and_fault_authorization(self):
        base=[sys.executable,str(Path(__file__).with_name('observe_driving_stop.py')),
              '--test','interlock-feedback-fault','--test-interlock','--operator-confirmed',
              '--run-dir','/tmp/not-used']
        for extra, expected in [([], 'explicit motion and operator confirmation'),
            (['--permit-motion-confirmed'], 'wheels-raised feedback fault requires explicit confirmation')]:
            result=subprocess.run(base+extra,capture_output=True,text=True)
            self.assertEqual(result.returncode,2)
            self.assertIn(expected,result.stderr)

    def test_feedback_fault_requires_fresh_bounded_measured_forward_motion(self):
        row={'topic':'platform/odom','elapsed':.25,'message':{'twist':{'twist':{
            'linear':{'x':.02,'y':0.},'angular':{'z':0.}}}}}
        command={'topic':'platform/motors/cmd','elapsed':.24,'message':{'velocity':[.1,.1,.1,.1]}}
        self.assertFalse(feedback_fault_due([row],10.1,10.,10.3,False))
        self.assertTrue(feedback_fault_due([row,command],10.1,10.,10.3,False))
        self.assertFalse(feedback_fault_due([row,command],None,10.,10.3,False))
        self.assertFalse(feedback_fault_due([row,command],10.1,10.,10.3,True))
        self.assertFalse(feedback_fault_due([row,command],10.1,10.,10.8,False))
        row['elapsed']=.05
        self.assertFalse(feedback_fault_due([row,command],10.1,10.,10.3,False))
        row['elapsed']=.25;row['message']['twist']['twist']['linear']['y']=.03
        self.assertFalse(feedback_fault_due([row,command],10.1,10.,10.3,False))

    def test_reconnect_refuses_unlocked_install_and_missing_device_before_robot_access(self):
        base=[sys.executable,str(Path(__file__).with_name('observe_driving_stop.py')),
              '--test','interlock-reconnect','--operator-confirmed','--run-dir','/tmp/not-used']
        for extra,expected in [([], 'requires the installed test interlock'),
                               (['--test-interlock'],'requires one --reconnect-device')]:
            result=subprocess.run(base+extra,capture_output=True,text=True)
            self.assertEqual(result.returncode,2)
            self.assertIn(expected,result.stderr)

    def test_disconnect_keeps_lease_for_observation_but_same_fixed_window_applies(self):
        self.assertFalse(cancel_permit_on_input_loss('interlock-disconnect'))
        self.assertTrue(cancel_permit_on_input_loss('interlock-permit'))
        self.assertFalse(cancel_permit_on_input_loss('interlock-permit','deadman'))
        self.assertFalse(cancel_permit_on_input_loss('interlock-permit','e-stop'))
        self.assertTrue(bounded_permit_window(10,10.8,False))
        self.assertFalse(bounded_permit_window(10,11.25,False))

    def test_interlocked_disconnect_requires_explicit_transport_authorization(self):
        result=subprocess.run([sys.executable,str(Path(__file__).with_name('observe_driving_stop.py')),
            '--test','interlock-disconnect','--test-interlock','--operator-confirmed',
            '--permit-motion-confirmed','--run-dir','/tmp/not-used'],capture_output=True,text=True)
        self.assertEqual(result.returncode,2)
        self.assertIn('requires a device and explicit moving-disconnect confirmation',result.stderr)

    def test_forward_command_allows_tiny_manual_yaw_but_not_changed_speed_or_lateral_limits(self):
        t={'linear':{'x':.05,'y':0.,'z':0.},'angular':{'x':0.,'y':0.,'z':.0014292835257947445}}
        self.assertTrue(forward_trial_command_safe(t))
        t['angular']['z']=.00615965249016881;self.assertTrue(forward_trial_command_safe(t))
        t['angular']['z']=.01;self.assertTrue(forward_trial_command_safe(t))
        t['angular']['z']=.010001;self.assertFalse(forward_trial_command_safe(t))
        t['angular']['z']=0.;t['linear']['y']=.001;self.assertFalse(forward_trial_command_safe(t))
        t['linear']['y']=0.;t['linear']['x']=.051;self.assertFalse(forward_trial_command_safe(t))
        t['linear']['x']=float('nan');self.assertFalse(forward_trial_command_safe(t))

    def test_permit_requires_input_is_bounded_and_cancellation_cannot_resume(self):
        self.assertFalse(bounded_permit_window(None,10,False))
        self.assertFalse(bounded_permit_window(10,9.9,False))
        self.assertTrue(bounded_permit_window(10,11.249,False))
        self.assertFalse(bounded_permit_window(10,11.25,False))
        self.assertFalse(bounded_permit_window(10,10.1,True))

    def test_motion_permit_requires_explicit_authorization_before_robot_access(self):
        result=subprocess.run([sys.executable,str(Path(__file__).with_name('observe_driving_stop.py')),
            '--test','interlock-permit','--test-interlock','--operator-confirmed','--run-dir','/tmp/not-used'],capture_output=True,text=True)
        self.assertEqual(result.returncode,2)
        self.assertIn('requires explicit motion and operator confirmation',result.stderr)

    def test_held_forward_requires_enable_and_finite_forward_not_sideways(self):
        m={'buttons':[0]*15,'axes':[0.,.2,0.,0.,0.,0.]}
        self.assertFalse(held_forward_input(m))
        m['buttons'][4]=1;self.assertTrue(held_forward_input(m))
        m['axes'][0]=.2;self.assertFalse(held_forward_input(m))
        m['axes'][0]=0;m['axes'][1]=float('nan');self.assertFalse(held_forward_input(m))
        self.assertFalse(held_forward_input({'buttons':[],'axes':[]}))

    def test_neutral_uses_deployed_motion_axes_not_unused_trigger_axes(self):
        m={'buttons':[0]*15,'axes':[0.,0.,1.,0.,0.,-1.]}
        row={'topic':'joy_teleop/joy','elapsed':1.,'message':m}
        self.assertTrue(controller_neutral([row],1.1))
        for axis in (0,1,3,4):
            m['axes'][axis]=.2
            self.assertFalse(controller_neutral([row],1.1))
            m['axes'][axis]=0.
        m['buttons'][4]=1;m['axes'][4]=.2
        self.assertTrue(held_forward_input(m))
        m['axes'][3]=.2;self.assertFalse(held_forward_input(m))

    def test_held_check_requires_interlocked_install_before_robot_access(self):
        result=subprocess.run([sys.executable,str(Path(__file__).with_name('observe_driving_stop.py')),
            '--test','interlock-held','--operator-confirmed','--run-dir','/tmp/not-used'],capture_output=True,text=True)
        self.assertEqual(result.returncode,2)
        self.assertIn('requires the installed test interlock',result.stderr)

    def test_disconnect_scheduler_waits_for_command_and_never_repeats(self):
        self.assertFalse(disconnect_due(None, 10, False))
        self.assertFalse(disconnect_due(2, 2.49, False))
        self.assertTrue(disconnect_due(2, 2.5, False))
        self.assertFalse(disconnect_due(2, 10, True))
    def test_automatic_disconnect_refuses_moving_mode_or_invalid_device_before_robot_access(self):
        for mode, device in [('disconnect', 'AC:36:1B:89:A1:0A'),
                             ('disconnect-stationary', 'invalid-device')]:
            result = subprocess.run([sys.executable,
                str(Path(__file__).with_name('observe_driving_stop.py')),
                '--test', mode, '--operator-confirmed', '--disconnect-device', device,
                '--run-dir', '/tmp/not-used'], capture_output=True, text=True)
            self.assertEqual(result.returncode, 2)
            self.assertIn('automatic moving disconnect is disabled' if mode == 'disconnect'
                          else 'automatic Bluetooth disconnect requires a supported test', result.stderr)

    def test_release_check_rejects_commands_and_nonfinite_or_moving_wheels(self):
        twist={'linear':{'x':0.,'y':0.},'angular':{'z':0.}}
        self.assertTrue(release_sample_safe('platform/cmd_vel', {'twist':twist}))
        twist['linear']['x']=.0001
        self.assertFalse(release_sample_safe('platform/cmd_vel', {'twist':twist}))
        self.assertTrue(release_sample_safe('platform/odom', {'twist':{'twist':twist}}))
        twist['linear']['x']=.002
        self.assertFalse(release_sample_safe('platform/odom', {'twist':{'twist':twist}}))
        self.assertFalse(release_sample_safe('platform/motors/cmd', {'velocity':[0.,float('nan')]}))
        self.assertFalse(release_sample_safe('platform/motors/cmd', {'velocity':[0.,.001]}))
        self.assertTrue(release_sample_safe('platform/motors/cmd', {'velocity':[0.]*4}))

    def test_released_then_one_asserted_report_does_not_arm(self):
        rows=[stop(0,False)];clock=[0.];states=iter([True,True]);calls=[]
        def spin(seconds):
            calls.append(seconds);clock[0]+=seconds
            rows.append(stop(clock[0],next(states)))
        await_pressed_stop(rows,spin,lambda:clock[0],1)
        self.assertEqual(len(calls),2)

    def test_stale_asserted_or_missing_reports_time_out(self):
        for rows in ([],[stop(0,True),stop(.1,True)]):
            clock=[10.]
            def spin(seconds):clock[0]+=seconds
            with self.assertRaisesRegex(RuntimeError,'never armed'):
                await_pressed_stop(rows,spin,lambda:clock[0],.5)

    def test_recent_asserted_reports_arm_without_wait(self):
        rows=[stop(0,True),stop(1,True)]
        await_pressed_stop(rows,lambda seconds:self.fail('unexpected wait'),lambda:1.2)

    def test_controller_must_be_fresh_neutral_and_not_enable_held(self):
        row={'topic':'joy_teleop/joy','elapsed':1.,
             'message':{'buttons':[0]*15,'axes':[0.]*6}}
        self.assertTrue(controller_neutral([row],1.1))
        self.assertFalse(controller_neutral([row],2))
        row['message']['buttons'][4]=1
        self.assertFalse(controller_neutral([row],1.1))
        row['message']['buttons'][4]=0;row['message']['axes'][1]=.2
        self.assertFalse(controller_neutral([row],1.1))
        row['message']['axes'][1]=float('nan')
        self.assertFalse(controller_neutral([row],1.1))
        self.assertFalse(controller_neutral([],1.1))


class DeadmanYawToleranceTests(unittest.TestCase):
    def test_runs_3_4_6_yaw_passes_only_with_the_deadman_tolerance(self):
        for yaw in (.0109, .011, .0109239):
            t={'linear':{'x':.05,'y':0.,'z':0.},'angular':{'x':0.,'y':0.,'z':yaw}}
            self.assertFalse(directional_trial_command_safe(t,'forward'))
            self.assertTrue(directional_trial_command_safe(t,'forward',DEADMAN_YAW_TOLERANCE))
        t={'linear':{'x':.05,'y':0.,'z':0.},'angular':{'x':0.,'y':0.,'z':.0201}}
        self.assertFalse(directional_trial_command_safe(t,'forward',DEADMAN_YAW_TOLERANCE))
        t={'linear':{'x':.05,'y':.001,'z':0.},'angular':{'x':0.,'y':0.,'z':.011}}
        self.assertFalse(directional_trial_command_safe(t,'forward',DEADMAN_YAW_TOLERANCE))
        t={'linear':{'x':.0501,'y':0.,'z':0.},'angular':{'x':0.,'y':0.,'z':.011}}
        self.assertFalse(directional_trial_command_safe(t,'forward',DEADMAN_YAW_TOLERANCE))


if __name__=='__main__':unittest.main()
