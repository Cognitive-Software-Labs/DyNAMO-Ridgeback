"""The two videos of one recorded demonstration run: complete and presentation.

Usage: make_demo_videos.py DEMO_DIR      (a record_demo_run.sh output folder)

record_demo.py saved one composite frame about every 0.5 s (Isaac view cameras, RViz and
the mission state), with its wall time and state in frames.csv. From those frames only:

- DEMO_DIR/complete.mp4: every frame, each shown for the real time until the next one, so
  the video keeps the run's original timing from the start of the recording to its end.
- DEMO_DIR/presentation.mp4: the same frames in the same order, with a banner on every
  frame naming the stage and its playback speed. Long stages (search, mapping around G1,
  the return) play faster, as the banner says; the parked wait plays in real time. Short
  title cards separate the stages. Nothing is taken from another run, reordered or removed:
  faster playback only drops frames within a stage.

DEMO_DIR/videos.txt records both durations and the speed of every stage.
"""
import csv
import os
import subprocess
import sys

from PIL import Image, ImageDraw, ImageFont

FPS = 25
# Stage of each mission state, its playback speed and its banner text.
STAGES = [
    ('start', 'Start in the hospital ward', 1.0, {'WAITING', 'WAITING_FOR_START_POSE'}),
    ('search', 'Autonomous exploration: searching for G1', 4.0, {'SEARCH_G1'}),
    ('locate', 'G1 detected: turning toward it and localizing it in depth', 1.0,
     {'STOPPING_EXPLORATION', 'RETURNING_TO_OBSERVATION', 'SETTLING_MOTION', 'LOCALIZE_G1', 'G1_LOCALIZED'}),
    ('map', 'Mapping the floor around G1', 4.0, {'EXPLORING_AROUND_G1'}),
    ('plan', 'Parking donut: candidate poses, route checks and the selected pose', 1.0,
     {'PARKING_CANDIDATES_READY', 'EVALUATING_PARKING_PATHS', 'PARKING_PATH_FEASIBLE'}),
    ('drive', 'Driving to the selected parking pose (LiDAR-checked)', 1.0,
     {'NAVIGATING_TO_PREPARKING', 'NAVIGATING_TO_PARKING'}),
    ('parked', 'Parked: the five-second wait', 1.0, {'PARKED'}),
    ('return', 'Returning to the start', 2.0, {'RETURNING_TO_START'}),
    ('end', 'Mission end', 1.0, {'MISSION_COMPLETE', 'MISSION_FAILED', 'PARKING_PLANNING_FAILED',
                                 'PARKING_EXECUTION_FAILED', 'MOTION_SETTLEMENT_FAILED', 'MISSION_CANCELED'}),
]
TITLE_S = 2.5


def stage_of(state):
    return next((s for s in STAGES if state in s[3]), ('other', state, 4.0, set()))


def font(size, bold=False):
    name = 'DejaVuSans-Bold.ttf' if bold else 'DejaVuSans.ttf'
    try:
        return ImageFont.truetype(name, size)
    except OSError:
        return ImageFont.load_default()


def banner(image, title, speed):
    draw = ImageDraw.Draw(image)
    w = image.width
    draw.rectangle((0, image.height - 64, w, image.height), fill=(20, 24, 30))
    draw.text((20, image.height - 50), title, fill=(240, 244, 248), font=font(30, True))
    text, colour = (('RECORDED SPEED', (80, 200, 120)) if speed == 1.0 else
                    (f'PLAYBACK {speed:g}x FASTER', (255, 170, 40)))
    draw.text((w - 20 - draw.textlength(text, font=font(30, True)), image.height - 50), text,
              fill=colour, font=font(30, True))
    return image


def title_card(size, lines, path):
    card = Image.new('RGB', size, (20, 24, 30))
    draw = ImageDraw.Draw(card)
    y = size[1] // 2 - 40 * len(lines) // 2 - 20
    for k, (text, big) in enumerate(lines):
        f = font(54 if big else 30, big)
        draw.text(((size[0] - draw.textlength(text, font=f)) / 2, y), text,
                  fill=(240, 244, 248) if big else (190, 200, 212), font=f)
        y += 80 if big else 48
    card.save(path)


def real_time_factor(run):
    """Simulated seconds per recorded (wall) second, from the recorded detection messages."""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import numpy as np
    from make_demo_storyboard import detections
    dets = detections(run)
    return float(np.polyfit([x[0] for x in dets], [x[1] for x in dets], 1)[0])


def encode(listing, out):
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'concat', '-safe', '0', '-i', listing,
                    '-vf', f'fps={FPS},scale=1920:-2,format=yuv420p', '-c:v', 'libx264', '-crf', '23',
                    '-preset', 'medium', out], check=True)


def duration(path):
    return float(subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'format=duration', '-of',
                                 'csv=p=0', path], capture_output=True, text=True, check=True).stdout)


def main():
    d = os.path.abspath(sys.argv[1])
    rows = [(int(i), float(t), s) for i, t, s in csv.reader(open(os.path.join(d, 'frames.csv')))]
    frames = [(os.path.join(d, 'frames', f'{i:06d}.jpg'), t, s) for i, t, s in rows]
    frames = [f for f in frames if os.path.exists(f[0])]
    result = open(os.path.join(d, 'run', 'result.txt')).read().split()
    outcome = result[2] if len(result) > 2 else 'unknown'
    commit = open(os.path.join(d, 'run', 'commit.txt')).readline().strip()[:7]
    size = Image.open(frames[0][0]).size
    rtf = real_time_factor(os.path.join(d, 'run'))
    notes = []
    if os.path.exists(os.path.join(d, 'notes.txt')):
        notes = [l.strip() for l in open(os.path.join(d, 'notes.txt')) if l.strip()]

    # A. Complete: every frame for the real time to the next one.
    with open(os.path.join(d, 'complete.txt'), 'w') as stream:
        for (path, t, _), (_, t_next, _) in zip(frames, frames[1:] + [(None, frames[-1][1] + 0.5, None)]):
            stream.write(f"file '{path}'\nduration {max(t_next - t, 0.04):.3f}\n")
        stream.write(f"file '{frames[-1][0]}'\n")
    encode(os.path.join(d, 'complete.txt'), os.path.join(d, 'complete.mp4'))

    # B. Presentation: the same frames, stage banners and speeds, title cards between stages.
    out = os.path.join(d, 'presentation_frames')
    os.makedirs(out, exist_ok=True)
    entries, stage_log, current = [], [], None
    intro = os.path.join(out, 'title_intro.png')
    title_card(size, [('Autonomous Ridgeback parking beside a Unitree G1', True),
                      ('Isaac Sim, hospital ward, scenario standard', False),
                      (f'One continuous run on the final code ({commit}; mission code b519ab0)', False),
                      (f'Isaac Sim ran at {rtf:.2f}x real time; the recording follows the wall clock', False),
                      ('Long stages play faster; every banner states the speed', False)],
               intro)
    entries.append((intro, 5.0))
    stage_time = {}
    for (path, t, state), (_, t_next, _) in zip(frames, frames[1:] + [(None, frames[-1][1] + 0.5, None)]):
        key, title, speed, _ = stage_of(state)
        if key != current:
            card = os.path.join(out, f'title_{len(entries):04d}.png')
            extra = []
            if key == 'parked':
                extra = [(f'the 5 s wait is simulated time; at {rtf:.2f}x real time it took about '
                          f'{5.0 / rtf:.0f} s of the recording', False)]
            title_card(size, [(title, True), ('at recorded speed' if speed == 1.0 else
                                              f'shown {speed:g}x faster than recorded', False)] + extra, card)
            entries.append((card, TITLE_S))
            current = key
            stage_log.append([key, title, speed, 0.0])
        real = max(t_next - t, 0.04)
        stage_log[-1][3] += real
        shown = os.path.join(out, os.path.basename(path).replace('.jpg', '.png'))
        banner(Image.open(path).convert('RGB'), title, speed).save(shown)
        entries.append((shown, real / speed))
    end = os.path.join(out, 'title_end.png')
    title_card(size, [(f'Outcome: {outcome}', True)] + [(n, False) for n in notes], end)
    entries.append((end, 8.0))
    with open(os.path.join(d, 'presentation.txt'), 'w') as stream:
        for path, seconds in entries:
            stream.write(f"file '{path}'\nduration {seconds:.3f}\n")
        stream.write(f"file '{entries[-1][0]}'\n")
    encode(os.path.join(d, 'presentation.txt'), os.path.join(d, 'presentation.mp4'))

    complete, presentation = duration(os.path.join(d, 'complete.mp4')), duration(os.path.join(d, 'presentation.mp4'))
    with open(os.path.join(d, 'videos.txt'), 'w') as stream:
        stream.write(f'run outcome {outcome}, code {commit}, simulation at {rtf:.2f}x real time\n')
        stream.write(f'complete.mp4: {complete:.1f} s, real time, {len(frames)} frames\n')
        stream.write(f'presentation.mp4: {presentation:.1f} s, same frames, title cards {TITLE_S} s\n')
        for key, title, speed, real in stage_log:
            stream.write(f'  {title}: {real:.1f} s recorded, shown at {speed:g}x ({real / speed:.1f} s)\n')
    print(open(os.path.join(d, 'videos.txt')).read())


if __name__ == '__main__':
    main()
