#!/usr/bin/env python3
"""Run standalone F1 motion/logging fixtures; no robot commands are published."""
import argparse
import csv
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import uuid
import hashlib

from ridgeback_leader_follow.crowd_appearance import SCENE_PROFILES
from ridgeback_leader_follow.crowd_scenarios import SCENARIOS, CROWD_SIZES, load_scenario

SAMPLE_DT = 0.05
FIELDS = ('sim_time', 'scenario_time', 'command_time', 'person', 'role',
          'frame', 'x', 'y', 'z', 'yaw', 'command_x', 'command_y', 'command_yaw')


def write_sample(writer, person, sim_time, scenario_time, command_time, observed):
    x, y, yaw = person.sample(command_time)
    writer.writerow(dict(zip(FIELDS, (
        sim_time, scenario_time, command_time, person.name, person.role, 'world',
        *observed, x, y, yaw))))


def stop_process(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def interrupt_run(_signum, _frame):
    """Make SIGTERM use the same owned-process cleanup as Ctrl-C."""
    raise KeyboardInterrupt('Scene interrupted')


def run_gazebo(scenario, duration, output, writer, gui, record=False, scene_profile='baseline', environment=None, render_quality='reference', tuning=None, drive=None):
    from ridgeback_leader_follow.crowd_gazebo import run
    return run(scenario, duration, output, writer, gui, record, scene_profile, environment, render_quality, tuning, drive)


def run_isaac(scenario, duration, output, writer, gui, finish, record=False, scene_profile="baseline", environment=None, render_quality="reference", tuning=None, drive=None):
    from ridgeback_leader_follow.crowd_isaac import run
    return run(scenario, duration, output, writer, gui, finish, record, scene_profile, environment, render_quality, tuning, drive)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', required=True, choices=('gz', 'isaac'))
    parser.add_argument('--scenario', default='crossing', choices=(*SCENARIOS,'leader_out_of_view','explainer'))
    parser.add_argument('--duration', type=float, help='Shorten the selected scenario')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--demo-cycles', type=int, default=0,
                        help='Repeat a continuous 30-second lab walk (1..120 cycles)')
    parser.add_argument('--scene-profile', choices=SCENE_PROFILES, default='baseline',
                        help='Lab uses a CC0 casual pedestrian and furnished corridor (Isaac only); '
                             'f3 uses distinct outfits; realistic adds textured worlds and Rocketbox pedestrians')
    parser.add_argument('--environment',choices=('modern_office','office','warehouse','courtyard'),help='Realistic-profile world; defaults to modern_office')
    parser.add_argument('--performance-profile',choices=('reference','balanced'),default='reference')
    parser.add_argument('--human-detail',choices=('full','distant'))
    parser.add_argument('--sensor-quality',choices=('reference','reduced'))
    parser.add_argument('--collision-quality',choices=('reference','simple'))
    parser.add_argument('--render-quality', choices=('reference', 'performance'), default='reference',
                        help='Performance simplifies lighting/reflections; preserves people, sensors and paths')
    parser.add_argument('--isaac-step-hz', type=int, choices=(30, 60),
                        help='Isaac updates per simulated second (default: recorded timing); '
                             '60 doubles viewport updates while sensors keep their tick rates')
    parser.add_argument('--drive', action='store_true',
                        help='Let cmd_vel move the robot (closed-loop following); '
                             'people wait at their start poses until a follow stack publishes')
    parser.add_argument('--real-time-factor', type=float, default=.5,
                        help='With --drive: simulated seconds per wall second (perception must keep up)')
    parser.add_argument('--gui', action='store_true', help='Open a simulator window')
    parser.add_argument('--crowd-size', type=int, choices=CROWD_SIZES, default=4)
    parser.add_argument('--record', action='store_true', help='Record sensor topics to a ROS bag')
    parser.add_argument('--ros-domain', type=int, help='Override the private ROS domain (0..232)')
    args = parser.parse_args(argv)
    from ridgeback_leader_follow.simulation_performance import resolve_tuning
    drive = None
    if args.drive:
        from ridgeback_leader_follow.simulation_performance import DriveOptions
        try:
            drive = DriveOptions(real_time_factor=args.real_time_factor)
        except ValueError as error:
            parser.error(str(error))
    if args.isaac_step_hz is not None and args.backend != 'isaac':
        parser.error('--isaac-step-hz requires --backend isaac')
    tuning=resolve_tuning(args.performance_profile,args.human_detail,args.sensor_quality,args.collision_quality,args.isaac_step_hz)
    if tuning != resolve_tuning() and args.scene_profile!='realistic':
        parser.error('runtime performance options require --scene-profile realistic')
    if args.render_quality != 'reference' and args.scene_profile != 'realistic':
        parser.error('--render-quality performance requires --scene-profile realistic')
    if args.environment is not None and args.scene_profile != "realistic":
        parser.error("--environment requires --scene-profile realistic")
    if args.scene_profile == "realistic":args.environment=args.environment or "modern_office"
    if args.scene_profile == 'lab' and args.backend != 'isaac':
        parser.error('The lab profile currently requires --backend isaac')
    if args.ros_domain is None:
        args.ros_domain = 100 + int(uuid.uuid4().hex[:8], 16) % 100
    if not 0 <= args.ros_domain <= 232:
        parser.error('ros-domain must be between 0 and 232')
    if args.scenario == 'explainer':
        if args.scene_profile!='realistic' or args.environment!='modern_office':
            parser.error('explainer requires realistic profile and modern_office')
        from ridgeback_leader_follow.explainer_motion import load_scenario as explainer_scenario
        scenario=explainer_scenario()
    elif args.scenario == 'leader_out_of_view':
        if args.scene_profile != 'f3':
            parser.error('leader_out_of_view requires --scene-profile f3')
        from ridgeback_leader_follow.recognition_scenarios import load_recognition_scenario
        scenario = load_recognition_scenario(args.scenario)
    else:
        scenario = load_scenario(args.scenario, args.crowd_size)
    if args.scene_profile == 'realistic':
        from ridgeback_leader_follow.realistic_scene import load_scenario as realistic_scenario
        scenario=realistic_scenario(args.scenario,args.crowd_size,args.environment)
    if args.demo_cycles:
        if args.scene_profile != 'lab' or args.duration is not None or not 1 <= args.demo_cycles <= 120:
            parser.error('demo-cycles requires lab, 1..120 cycles, and no duration override')
        from ridgeback_leader_follow.lab_scene import demo_scenario
        scenario = demo_scenario(args.demo_cycles)
    duration = args.duration if args.duration is not None else scenario.duration
    if (not math.isfinite(duration) or not SAMPLE_DT <= duration <= scenario.duration
            or not math.isclose(duration / SAMPLE_DT, round(duration / SAMPLE_DT))):
        parser.error('duration must be a multiple of 0.05 seconds within the scenario duration')
    # Relative output paths are resolved once, before any subprocess starts.
    output = (args.output_dir or Path('artifacts/leader-following') /
              datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')).resolve()
    output.mkdir(parents=True, exist_ok=False)
    # Preserve source inputs even for tests of an uncommitted implementation.
    shutil.copytree(Path(__file__).resolve().parent, output / 'source',
                    ignore=shutil.ignore_patterns('__pycache__'))
    manifest = {'scenario': asdict(scenario), 'backend': args.backend,
                'duration': duration, 'sample_dt': SAMPLE_DT, 'status': 'running',
                'scene_profile': args.scene_profile,
                'geometry': ('CC0 Innerscene Maya, furnished lab corridor' if args.scene_profile == 'lab'
                             else 'Cesium Man CC-BY-4.0 skinned animation; baseline corridor'),
                'crowd_size': (10 if scenario.name=='explainer' else args.crowd_size), 'ros_domain': args.ros_domain,
                'runtime_tuning':asdict(tuning), 'camera_contract':asdict(tuning.camera_profile()), 'render_quality': args.render_quality, 'drive': asdict(drive) if drive else None, 'frame': 'world', 'floor_z': 0., 'output_dir': str(output)}
    if args.scene_profile == 'f3':
        from ridgeback_leader_follow.f3_scene import GEOMETRY, manifest_entry
        # Evidence for scoring only; appearance identities are never published.
        manifest.update(geometry=GEOMETRY, appearance=manifest_entry(scenario))
    if args.scene_profile == 'realistic':
        from ridgeback_leader_follow.realistic_scene import manifest_entry
        manifest.update(manifest_entry(scenario,args.environment))
        from ridgeback_leader_follow.simulation_performance import use_distance_lod
        manifest['human_geometry']={person.name:('lod1' if use_distance_lod(person,tuning) else 'full') for person in scenario.people}
    (output / 'setup').mkdir()
    import yaml
    declaration=yaml.safe_load((Path.cwd()/'clearpath/robot.yaml').read_text())
    (output/'setup/robot.yaml').write_text(yaml.safe_dump(tuning.setup_robot(declaration),sort_keys=False))
    os.environ['ROS_DOMAIN_ID'] = str(args.ros_domain)
    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    from ament_index_python.packages import get_package_share_directory, get_package_prefix
    package = Path(get_package_share_directory('ridgeback_leader_follow'))
    shutil.copytree(package / 'sim', output / 'simulation_inputs')
    shutil.copytree(package / 'launch', output / 'launch_inputs')
    library = Path(get_package_prefix('ridgeback_leader_follow')) / 'lib/ridgeback_leader_follow/libCrowdMotion.so'
    if library.is_file():
        shutil.copyfile(library, output / 'simulation_inputs/libCrowdMotion.so')
    if args.backend=='isaac':
        adapter=Path(get_package_share_directory('ridgeback_autonomy_isaac'))/'sim/isaac'
        shutil.copytree(adapter,output/'backend_inputs',ignore=shutil.ignore_patterns('usd','__pycache__'))
    manifest['input_sha256'] = {str(p.relative_to(output)): hashlib.sha256(p.read_bytes()).hexdigest()
        for directory in ('source','simulation_inputs','launch_inputs','setup','backend_inputs')
        for p in (output / directory).rglob('*') if p.is_file()}
    manifest['base_revision'] = subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
    (output / 'package.diff').write_bytes(subprocess.check_output(['git','diff','--binary','--','src/ridgeback_leader_follow']))
    os.environ.setdefault('CYCLONEDDS_URI', 'file://' + get_package_share_directory('ridgeback_autonomy') + '/config/cyclonedds.xml')
    os.environ.setdefault('ROS_LOG_DIR', str(output / 'ros'))
    manifest_path = output / 'manifest.json'
    manifest_path.write_text(json.dumps(manifest, indent=2) + '\n')
    print(f'Running {scenario.name} on {args.backend}; evidence: {output}', flush=True)
    previous_sigterm = signal.signal(signal.SIGTERM, interrupt_run)
    try:
        with (output / 'ground_truth.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS)
            writer.writeheader()
            if args.backend == 'gz':
                # Keep this fixture separate from every other Gazebo session.
                os.environ['GZ_PARTITION'] = f'leader_follow_{uuid.uuid4().hex}'
                run_gazebo(scenario, duration, output, writer, args.gui, args.record, args.scene_profile, args.environment, args.render_quality, tuning, drive)
            else:
                # Kit has its own argument parser; it must not parse this CLI.
                sys.argv = [sys.argv[0]]
                def finish(error=None):
                    stream.flush()
                    os.fsync(stream.fileno())
                    manifest['status'] = 'complete' if error is None else 'failed'
                    if error is not None:
                        manifest['error'] = f'{type(error).__name__}: {error}'
                    manifest_path.write_text(json.dumps(manifest, indent=2) + '\n')
                run_isaac(scenario, duration, output, writer, args.gui, finish, args.record, args.scene_profile, args.environment, args.render_quality, tuning, drive)
        manifest['status'] = 'complete'
    except BaseException as exc:
        manifest.update(status='failed', error=f'{type(exc).__name__}: {exc}')
        raise
    finally:
        manifest_path.write_text(json.dumps(manifest, indent=2) + '\n')
        signal.signal(signal.SIGTERM, previous_sigterm)
    print('Scene completed and simulator stopped.', flush=True)


if __name__ == '__main__':
    main()
