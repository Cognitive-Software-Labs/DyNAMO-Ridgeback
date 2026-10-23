#!/usr/bin/env python3
"""Compile the offline per-field age policy; never access ROS, CAN or hardware."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = args.output.resolve()
    evidence = ROOT/'artifacts/hardware'
    if root == evidence or not root.is_relative_to(evidence):
        parser.error('use a fresh directory under artifacts/hardware')
    root.mkdir(parents=True, exist_ok=False)
    header = Path(__file__).with_name('puma_field_freshness.hpp')
    (root/header.name).write_bytes(header.read_bytes())
    (root/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    source = r'''#include "puma_field_freshness.hpp"
#include <array>
#include <cassert>
#include <iostream>
using Policy = dynamo::PumaFieldFreshness;
using Field = Policy::Field;
constexpr std::int64_t ms = 1000000;
constexpr std::int64_t initial = 1000 * ms;
unsigned checks = 0;
void expect(bool value) {assert(value); ++checks;}
std::size_t width(unsigned f) {return f < 2 ? 2 : 4;}
void fill(Policy & p, std::int64_t stamp) {
  for (unsigned f = 0; f < Policy::Count; ++f)
    expect(p.receive(static_cast<Field>(f), stamp, width(f)));
}
int main() {
  bool threw = false;
  try {Policy invalid(0);} catch (const std::invalid_argument &) {threw = true;}
  expect(threw);
  Policy p(200 * ms);
  expect(!p.ready(initial)); expect(p.oldest_age_ns(initial) == -1);
  fill(p, initial);
  expect(p.ready(initial)); expect(p.ready(initial + 200 * ms));
  expect(!p.ready(initial + 200 * ms + 1)); expect(!p.ready(initial - 1));
  p.published(); expect(!p.ready(initial));
  expect(p.receive(Policy::Position, initial + 50 * ms, 4));
  expect(!p.ready(initial + 50 * ms));
  expect(p.receive(Policy::Speed, initial + 50 * ms, 4));
  expect(p.ready(initial + 50 * ms));
  expect(p.oldest_age_ns(initial + 50 * ms) == 50 * ms);
  // Diagnostics and repeated readiness inspection do not consume updates.
  expect(p.ready(initial + 50 * ms)); expect(p.ready(initial + 50 * ms));
  for (unsigned missing_wheel = 0; missing_wheel < 4; ++missing_wheel) {
    for (unsigned missing_field = 0; missing_field < Policy::Count; ++missing_field) {
      std::array<Policy, 4> wheels{{Policy(200*ms), Policy(200*ms), Policy(200*ms), Policy(200*ms)}};
      for (unsigned wheel = 0; wheel < 4; ++wheel)
        for (unsigned f = 0; f < Policy::Count; ++f)
          if (wheel != missing_wheel || f != missing_field)
            expect(wheels[wheel].receive(static_cast<Field>(f), initial, width(f)));
      bool all = true;
      for (const auto & wheel : wheels) all = all && wheel.ready(initial + 1);
      expect(!all);
    }
  }
  for (unsigned field = 0; field < Policy::Count; ++field) {
    Policy damaged(200*ms); fill(damaged, initial);
    expect(!damaged.receive(static_cast<Field>(field), initial + 1, width(field)-1));
    expect(!damaged.ready(initial + 1));
    expect(damaged.receive(static_cast<Field>(field), initial + 2, width(field)));
    expect(damaged.ready(initial + 2));
  }
  Policy queued(200*ms); fill(queued, initial);
  // Delayed decode/publication cannot make old ingress stamps fresh.
  expect(!queued.ready(initial + 201*ms));
  Policy ageing(200*ms); fill(ageing, initial);
  for (int tick = 1; tick <= 5; ++tick) {
    ageing.published();
    ageing.receive(Policy::Position, initial + tick*50*ms, 4);
    ageing.receive(Policy::Speed, initial + tick*50*ms, 4);
    expect(ageing.ready(initial + tick*50*ms) == (tick <= 4));
  }
  Policy order(200*ms); fill(order, initial);
  expect(!order.receive(Policy::Speed, initial - 1, 4)); expect(!order.ready(initial));
  expect(!order.receive(Policy::Speed, initial - 2, 4));
  expect(order.receive(Policy::Speed, initial + 1, 4)); expect(order.ready(initial + 1));
  order.clear(); expect(!order.ready(initial));
  expect(!order.receive(Policy::Speed, 0, 4));
  bool rejected = false;
  try {Policy inverted(200*ms, 100*ms);} catch (const std::invalid_argument &) {rejected = true;}
  expect(rejected);
  // Split bounds: motion 150 ms must be new each publication; auxiliary 500 ms.
  Policy split(150*ms, 500*ms); fill(split, initial);
  expect(split.ready(initial + 150*ms)); expect(!split.ready(initial + 150*ms + 1));
  for (int tick = 1; tick <= 11; ++tick) {
    split.published();
    split.receive(Policy::Position, initial + tick*50*ms, 4);
    split.receive(Policy::Speed, initial + tick*50*ms, 4);
    expect(split.motion_age_ns(initial + tick*50*ms) == 0);
    expect(split.auxiliary_age_ns(initial + tick*50*ms) == tick*50*ms);
    expect(split.ready(initial + tick*50*ms) == (tick <= 10));
  }
  Policy damaged_aux(150*ms, 500*ms); fill(damaged_aux, initial);
  expect(!damaged_aux.receive(Policy::Setpoint, initial + 1, 2));
  expect(!damaged_aux.ready(initial + 1)); expect(damaged_aux.auxiliary_age_ns(initial + 1) == -1);
  std::cout << checks << " policy assertions passed\n";
}
'''
    (root/'check.cpp').write_text(source)
    command = ['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-O0',
               '-fsanitize=undefined', '-fno-sanitize-recover=all',
               str(root/'check.cpp'), '-o', str(root/'check')]
    build = subprocess.run(command, capture_output=True, text=True)
    (root/'compiler.log').write_text(build.stdout+build.stderr)
    build.check_returncode()
    run = subprocess.run([str(root/'check')], capture_output=True, text=True, check=True)
    (root/'execution.log').write_text(run.stdout+run.stderr)
    report = {'status': 'offline-field-age-policy-verified-not-integrated',
              'stdout': run.stdout.strip(),
              'header_sha256': hashlib.sha256(header.read_bytes()).hexdigest(),
              'test_age_bound_ms': 200, 'split_bounds_tested_ms': {'motion': 150, 'auxiliary': 500}, 'production_age_bound_selected': False,
              'live_ros_or_can_access': False,
              'limits': 'Speed-mode receipt metadata only. Driver ingress/decoder integration, full package build, diagnostic wiring, loaded/moving captures and deployment qualification remain open.'}
    (root/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
