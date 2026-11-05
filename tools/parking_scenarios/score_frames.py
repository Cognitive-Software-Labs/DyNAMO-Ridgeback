"""Score saved camera frames with the mission's OWLv2 checkpoint below its threshold.

Usage (perception venv python): score_frames.py FRAMES_DIR [LABEL ...] > scores.jsonl
One JSON line per frame: stamp and every box above 0.05 per label, so the margin between
G1 and everything else in view can be read offline. Review only.
"""
import glob, json, os, sys
from PIL import Image
from transformers import pipeline

frames = sorted(glob.glob(os.path.join(sys.argv[1], '*.jpg')) + glob.glob(os.path.join(sys.argv[1], '*.png')))
labels = sys.argv[2:] or ['humanoid robot']
detect = pipeline('zero-shot-object-detection', model='google/owlv2-base-patch16-ensemble', device='cuda')
for path in frames:
    image = Image.open(path).convert('RGB')
    boxes = detect(image, candidate_labels=labels, threshold=0.05)
    print(json.dumps({'stamp_ns': int(os.path.basename(path).split('.')[0]), 'size': image.size,
                      'boxes': [{'label': b['label'], 'score': round(b['score'], 3),
                                 'box': [b['box'][k] for k in ('xmin', 'ymin', 'xmax', 'ymax')]}
                                for b in boxes]}), flush=True)
