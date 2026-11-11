"""Measured time in each mission state, from the mission manager's own transition log.

Usage: state_timing.py RUN_DIR [RUN_DIR ...]   (each with console.log)
Every 'Mission state transition: A -> B' line is stamped by the ROS logger; a state's
duration is the time from the transition into it to the transition out of it. States
still active when a run stopped are not counted. Prints a Markdown table per simulator.
"""
import collections
import os
import re
import statistics
import sys

TRANSITION = re.compile(r'\[(\d+\.\d+)\] \[[^\]]*mission_manager_node\]: '
                        r'Mission state transition: (\w+) -> (\w+);')


def durations(run):
    log = open(os.path.join(run, 'console.log'), errors='replace').read()
    steps = [(float(t), a, b) for t, a, b in TRANSITION.findall(log)]
    out = collections.defaultdict(list)
    for (t0, _, state), (t1, _, _) in zip(steps, steps[1:]):
        out[state].append(t1 - t0)
    return out


def main():
    table = collections.defaultdict(lambda: collections.defaultdict(list))
    runs = collections.Counter()
    for run in sys.argv[1:]:
        result = open(os.path.join(run, 'result.txt')).read().split() if os.path.exists(
            os.path.join(run, 'result.txt')) else ['?', 'gz']
        sim = result[1]
        runs[sim] += 1
        for state, values in durations(run).items():
            table[sim][state] += values
    for sim, states in table.items():
        print(f'\n{sim}: {runs[sim]} runs\n')
        print('| State | Visits | Median s | Min s | Max s |\n|---|---|---|---|---|')
        for state, values in sorted(states.items(), key=lambda kv: -statistics.median(kv[1])):
            print(f'| `{state}` | {len(values)} | {statistics.median(values):.1f} | '
                  f'{min(values):.1f} | {max(values):.1f} |')


if __name__ == '__main__':
    main()
