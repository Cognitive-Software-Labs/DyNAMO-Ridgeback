#!/usr/bin/env python3
"""Score an offline F2 candidate prediction file against an F1 truth log."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

from ridgeback_leader_follow.benchmarking.scoring import Position, score_frames


def evaluate(run, predictions, output, candidate, gate=.75, timestamp_tolerance=.001):
    run,predictions,output = map(lambda p: Path(p).resolve(),(run,predictions,output))
    manifest = json.loads((run/'manifest.json').read_text())
    if manifest.get('status') != 'complete':
        raise ValueError('Only complete F1 runs may be evaluated')
    truth = {}
    with (run/'ground_truth.csv').open() as stream:
        for row in csv.DictReader(stream):
            if row['frame'] != 'world':
                raise ValueError('Truth must use world coordinates')
            truth.setdefault(float(row['scenario_time']),[]).append(Position(
                row['person'],float(row['x']),float(row['y']),row['role']))
    stamps = sorted(truth)
    if not stamps:
        raise ValueError('No truth timestamps')
    predicted = {}
    with predictions.open() as stream:
        for line in stream:
            frame = json.loads(line)
            stamp = float(frame['scenario_time'])
            if frame.get('frame') != 'world' or not math.isfinite(stamp):
                raise ValueError('Predictions require finite scenario_time and world frame')
            # Input is one prediction frame per truth tick. Never silently
            # discard out-of-range predictions or overwrite duplicate frames.
            import bisect
            index = bisect.bisect_left(stamps,stamp)
            choices = stamps[max(0,index-1):min(len(stamps),index+1)]
            matched = min(choices,key=lambda t:abs(t-stamp))
            if abs(matched-stamp) > timestamp_tolerance or matched in predicted:
                raise ValueError('Unaligned or duplicate prediction timestamp')
            predicted[matched] = [Position(str(p['track_id']),float(p['x']),float(p['y']))
                                  for p in frame['people']]
    result = score_frames(((t,truth[t],predicted.get(t,[])) for t in stamps),gate)
    result.update(candidate=candidate,backend=manifest['backend'],
                  scenario=manifest['scenario']['name'],truth_frames=len(stamps),
                  prediction_frames=len(predicted),
                  truth_sha256=hashlib.sha256((run/'ground_truth.csv').read_bytes()).hexdigest(),
                  predictions_sha256=hashlib.sha256(predictions.read_bytes()).hexdigest())
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('x') as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
        stream.write('\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',required=True,type=Path)
    parser.add_argument('--predictions',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--candidate',required=True)
    parser.add_argument('--gate',type=float,default=.75)
    args = parser.parse_args()
    print(json.dumps(evaluate(args.run,args.predictions,args.output,args.candidate,args.gate),indent=2))


if __name__ == '__main__':
    main()
