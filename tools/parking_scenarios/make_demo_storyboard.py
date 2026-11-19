"""A demonstration storyboard from the recorded final-code hospital-ward runs: stills, not video.

Usage: make_demo_storyboard.py OUT_DIR

No final-code run was recorded as video, so this assembles the runs' own captures, in mission
order, into OUT_DIR/storyboard/NN_*.png (each with a caption bar naming the run, its code, the
mission state the capture was taken in and the mission time), OUT_DIR/storyboard.mp4 (each still
held STILL_S seconds: not real time, and the intervals between stills are omitted) and
OUT_DIR/README.md listing every source file. Captures are the run's RViz stage screenshots,
Gazebo server views and the robot camera frames run_scenario.sh saved (FRAMES=1). A camera frame
is placed by its simulation stamp, mapped to the mission log's wall clock through the recorded
detection messages (both stamps); a detection box is drawn only from the recorded detector
output for that frame (the frame saver keeps one frame every 2 s, so the frame shown is one the
detector also scored, after the robot turned toward the confirmed detection). Each still is checked against the mission state its caption names.
"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import make_results_page as results  # noqa: E402

STILL_S = 4
SIZE = (1280, 800)
CAPTION_H = 96
STANDARD = 'artifacts/exploration/hospital_2026-11-18/gz_standard_final'
REFUSAL = 'artifacts/exploration/hospital_2026-11-18/gz_no_feasible_final'
DETECTIONS = '/r100_0001/detections/target/raw'

# (title, run, kind, reference, accepted states). kinds: rviz (stage screenshot), simview
# (Gazebo view in the manifest), frame (robot camera frame at the first transition into the
# named state; 'box' draws the recorded detection box).
STEPS = [
    ('Ridgeback begins its search', STANDARD, 'rviz', 'EXPLORING', {'SEARCH_G1'}),
    ('G1 detected: the robot camera after turning toward it', STANDARD, 'frame', ('SETTLING_MOTION', 'box'),
     {'SETTLING_MOTION'}),
    ('G1 localized; the mission maps the floor around it', STANDARD, 'rviz', 'G1_LOCALIZED',
     {'G1_LOCALIZED', 'EXPLORING_AROUND_G1'}),
    ('Parking donut and the selected pose', STANDARD, 'rviz', 'NAVIGATING_TO_PREPARKING',
     {'NAVIGATING_TO_PREPARKING'}),
    ('Attempt 1 under way: the LiDAR check cancelled it 0.8 s later', STANDARD, 'rviz', 'NAVIGATING_TO_PARKING',
     {'NAVIGATING_TO_PARKING'}),
    ('Parked: attempt 2, G1\'s left side (angled view)', STANDARD, 'simview', 'gz_standard_parked_angled.png',
     {'PARKED'}),
    ('The five-second wait (RViz)', STANDARD, 'rviz', 'PARKED', {'PARKED'}),
    ('Back at the start', STANDARD, 'simview', 'gz_standard_end_overview.png', {'MISSION_COMPLETE'}),
    ('No feasible position: the pen around G1', REFUSAL, 'simview', 'gz_no_feasible_end_angled.png',
     {'MISSION_FAILED'}),
    ('No feasible position: every donut ray red, no attempt, returned', REFUSAL, 'rviz', 'END_MISSION_FAILED',
     {'MISSION_FAILED'}),
]


def detections(run_path):
    """[(wall_s, sim_s, boxes)] of the run's recorded detector output, in wall order."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=os.path.join(run_path, 'bag'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[DETECTIONS]))
    kind = get_message('ridgeback_interfaces/msg/TargetDetections')
    out = []
    while reader.has_next():
        _, data, wall_ns = reader.read_next()
        msg = deserialize_message(data, kind)
        boxes = [list(msg.bbox_xyxy[i:i + 4]) for i in range(0, len(msg.bbox_xyxy), 4)]
        out.append((wall_ns * 1e-9, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, boxes,
                    list(msg.scores)))
    return out


def camera_frame(r, state, box, window_s=10.0, match_s=0.05):
    """(path, wall time, note, boxes) of a saved camera frame from the ``window_s`` seconds after
    the first transition into ``state``. With ``box``, the frame must match (within ``match_s`` of
    simulation time) a recorded detection with a box, and the highest-scoring such frame is used."""
    when = next(t for t, _, new, _ in r['transitions'] if new == state)
    dets = [d for d in detections(r['path'])
            if when <= d[0] <= when + window_s and results.state_at(r, d[0]) == state]
    frames = sorted((int(f.split('.')[0]) * 1e-9, f) for f in os.listdir(os.path.join(r['path'], 'frames'))
                    if f.endswith('.jpg'))
    best = None
    for wall, sim, boxes, scores in dets:
        if box and not boxes:
            continue
        stamp, name = min(frames, key=lambda f: abs(f[0] - sim))
        if abs(stamp - sim) > match_s:
            continue
        score = max(scores) if scores else 0.0
        if best is None or score > best[0]:
            best = (score, name, wall + (stamp - sim), boxes)
    if best is None:
        raise SystemExit(f'no saved camera frame matches a recorded detection before {state}')
    score, name, wall, boxes = best
    note = f'box: the detector\'s recorded output for this frame, score {score:.2f}' if box else ''
    return os.path.join(r['path'], 'frames', name), wall, note, boxes if box else []


def still(r, kind, ref):
    shots = os.path.join(r['path'], 'shots')
    if kind == 'rviz':
        src, when = results.capture(r, {'kind': 'rviz', 'ref': ref, 'states': set()})
        return src, when, '', []
    if kind == 'simview':
        entry = next(e for e in json.load(open(os.path.join(shots, 'manifest.json'))) if e['file'] == ref)
        return os.path.join(shots, ref), float(entry['time']), 'Gazebo server view', []
    state, box = ref
    return camera_frame(r, state, box == 'box')


def compose(src, boxes, title, caption, out):
    from PIL import Image, ImageDraw, ImageFont
    image = Image.open(src).convert('RGB')
    if boxes:
        draw = ImageDraw.Draw(image)
        for x0, y0, x1, y1 in boxes:
            draw.rectangle((x0, y0, x1, y1), outline=(255, 170, 0), width=3)
    w, h = SIZE[0], SIZE[1] - CAPTION_H
    scale = min(w / image.width, h / image.height)
    image = image.resize((int(image.width * scale), int(image.height * scale)))
    canvas = Image.new('RGB', SIZE, (24, 28, 34))
    canvas.paste(image, ((w - image.width) // 2, (h - image.height) // 2))
    draw = ImageDraw.Draw(canvas)
    try:
        big = ImageFont.truetype('DejaVuSans-Bold.ttf', 28)
        small = ImageFont.truetype('DejaVuSans.ttf', 16)
    except OSError:
        big = small = ImageFont.load_default()
    draw.text((24, h + 12), title, fill=(240, 244, 248), font=big)
    draw.text((24, h + 54), caption, fill=(190, 200, 212), font=small)
    canvas.save(out)


def main():
    out = os.path.abspath(sys.argv[1])
    os.makedirs(os.path.join(out, 'storyboard'), exist_ok=True)
    runs = {p: results.load(p) for p in {s[1] for s in STEPS}}
    rows, files = [], []
    for n, (title, run, kind, ref, states) in enumerate(STEPS, 1):
        r = runs[run]
        src, when, note, boxes = still(r, kind, ref)
        state = results.state_at(r, when)
        if state not in states:
            raise SystemExit(f'step {n} {title}: captured during {state}, not {sorted(states)}')
        t = results.clock(when - r['transitions'][0][0])
        source = {'rviz': 'RViz screenshot', 'simview': 'Gazebo server view', 'frame': 'robot camera frame'}[kind]
        caption = (f'{r["name"]} · code {r["m"]["commit"]} (mission code b519ab0) · {source} · state {state} · '
                   f'{t} into the mission · still')
        name = os.path.join(out, 'storyboard', f'{n:02d}_{kind}.png')
        compose(src, boxes, f'{n}. {title}', caption, name)
        files.append(name)
        rows.append(f'| {n} | {title} | `{r["name"]}` | {state} | {t} | `{os.path.relpath(src, results.ROOT)}`'
                    f'{" (" + note + ")" if note else ""} |')
    concat = os.path.join(out, 'storyboard.txt')
    with open(concat, 'w') as stream:
        for name in files:
            stream.write(f"file '{name}'\nduration {STILL_S}\n")
        stream.write(f"file '{files[-1]}'\n")
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'concat', '-safe', '0', '-i', concat,
                    '-vf', 'fps=25,format=yuv420p', '-c:v', 'libx264', '-crf', '20',
                    os.path.join(out, 'storyboard.mp4')], check=True)
    with open(os.path.join(out, 'README.md'), 'w') as stream:
        stream.write(
            '# Final-code demonstration storyboard\n\n'
            'Built by `tools/parking_scenarios/make_demo_storyboard.py` from the captures of two '
            'final-code Gazebo runs in the hospital ward (mission code b519ab0): `gz_standard_final` '
            '(MISSION_COMPLETE) and `gz_no_feasible_final` (MISSION_FAILED, no_valid_candidate). '
            f'`storyboard.mp4` shows each still for {STILL_S} s: it is not real time, and everything '
            'between the stills is omitted. Mission times come from the mission log; every still was '
            'checked against the mission state its caption names. The only drawing added is the '
            'detector\'s own recorded box on the camera frame of step 2 and the caption bars.\n\n'
            '| Step | Shows | Run | Mission state | Mission time | Source |\n|---|---|---|---|---|---|\n'
            + '\n'.join(rows) + '\n')
    print('wrote', out, len(files), 'stills')


if __name__ == '__main__':
    main()
