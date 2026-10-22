#!/usr/bin/env python3
"""Exercise actual candidate aggregate routines with inert wheels/publisher."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from verify_puma_freshness_patch import ROOT, function


def verify(package, output):
    output = output.resolve()
    evidence = ROOT/'artifacts/hardware'
    if output == evidence or not output.is_relative_to(evidence):
        raise ValueError('use a fresh directory under artifacts/hardware')
    output.mkdir(parents=True, exist_ok=False)
    node_source = (package/'src/multi_puma_node.cpp').read_text()
    header = package/'include/puma_motor_driver/puma_field_freshness.hpp'
    shutil.copy2(header, output/header.name)
    program = r'''#include "puma_field_freshness.hpp"
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <thread>
#define RCLCPP_INFO(...) do {} while (0)
int64_t now_ns() {return std::chrono::duration_cast<std::chrono::nanoseconds>(
  std::chrono::steady_clock::now().time_since_epoch()).count();}
using Policy = dynamo::PumaFieldFreshness;
namespace can_hardware {struct Frame {unsigned field = 0;};}
unsigned checks = 0;
void expect(bool result) {if (!result) throw std::runtime_error("aggregate assertion failed"); ++checks;}
struct Driver {
  Policy fields{200000000};
  unsigned reads = 0, consumed = 0;
  bool feedbackReady(int64_t now) const {return fields.ready(now);}
  void feedbackPublished() {++consumed; fields.published();}
  void processMessage(const can_hardware::Frame & frame, int64_t ingress) {
    fields.receive(static_cast<Policy::Field>(frame.field), ingress, frame.field < 2 ? 2 : 4);
  }
  int deviceNumber() const {return 5;}
  std::string deviceName() const {return "offline";}
  double lastDutyCycle() {++reads; return 0;}
  double lastCurrent() {++reads; return 0;}
  double lastPosition() {++reads; return 0;}
  double lastSpeed() {++reads; return 0;}
  double lastSetpoint() {++reads; return 0;}
  void fill(int absent = -1, int64_t stamp = 0) {
    if (!stamp) stamp = now_ns();
    for (int f = 0; f < static_cast<int>(Policy::Count); ++f)
      if (f != absent) fields.receive(static_cast<Policy::Field>(f), stamp, f < 2 ? 2 : 4);
  }
};
namespace clearpath_motor_msgs {namespace msg {
struct PumaFeedback {int device_number; std::string device_name; double duty_cycle,current,travel,speed,setpoint;};
}}
struct Clock {int now() const {return 0;}};
struct Publisher {
  unsigned writes = 0;
  std::function<void()> before;
  template<class T> void publish(const T &) {if (before) before(); ++writes;}
};
struct MultiPumaNode {
  std::array<Driver, 4> drivers_;
  struct {
    std::array<clearpath_motor_msgs::msg::PumaFeedback, 4> drivers_feedback;
    struct {int stamp = 0;} header;
  } feedback_msg_;
  Clock clock;
  Clock * get_clock() {return &clock;}
  std::shared_ptr<Publisher> feedback_pub_ = std::make_shared<Publisher>();
  struct ReceivedFrame {can_hardware::Frame frame; int64_t ingress_ns = 0;};
  std::queue<ReceivedFrame> recv_msg_queue_;
  std::mutex recv_msg_mutex_;
  int64_t queue_timing_started_ns_ = 0;
  int64_t queue_age_max_ns_ = 0;
  int64_t queue_age_sum_ns_ = 0;
  uint64_t queue_age_count_ = 0;
  uint64_t queue_age_over_100ms_ = 0;
  bool getFeedback();
  void publishFeedback();
  void frameCallback(const can_hardware::Frame &);
  void processQueuedFrames();
};
'''
    program += function(node_source, 'bool MultiPumaNode::getFeedback()')+'\n'
    program += function(node_source, 'void MultiPumaNode::publishFeedback()')+'\n'
    program += function(node_source, 'void MultiPumaNode::frameCallback(')+'\n'
    program += function(node_source, 'void MultiPumaNode::processQueuedFrames()')+'\n'
    program += r'''int main() {
  for (unsigned missing = 0; missing < 4; ++missing) {
    for (int absent = 0; absent < static_cast<int>(Policy::Count); ++absent) {
      MultiPumaNode node;
      for (unsigned wheel = 0; wheel < 4; ++wheel)
        node.drivers_[wheel].fill(wheel == missing ? absent : -1);
      node.publishFeedback();
      expect(node.feedback_pub_->writes == 0);
      for (const auto & wheel : node.drivers_) expect(wheel.consumed == 0 && wheel.reads == 0);
      // Complete only the absent receipt; other wheels retain their updates.
      node.drivers_[missing].fields.receive(static_cast<Policy::Field>(absent), now_ns(), absent < 2 ? 2 : 4);
      node.publishFeedback(); expect(node.feedback_pub_->writes == 1);
      for (const auto & wheel : node.drivers_) expect(wheel.consumed == 1);
      node.publishFeedback(); expect(node.feedback_pub_->writes == 1);
    }
  }
  MultiPumaNode order;
  for (auto & wheel : order.drivers_) wheel.fill();
  order.feedback_pub_->before = [&] {
    for (const auto & wheel : order.drivers_)
      expect(wheel.consumed == 0 && wheel.feedbackReady(now_ns()));
  };
  order.publishFeedback(); expect(order.feedback_pub_->writes == 1);
  MultiPumaNode failure;
  for (auto & wheel : failure.drivers_) wheel.fill();
  failure.feedback_pub_->before = [] {throw std::runtime_error("synthetic publish failure");};
  bool threw = false;
  try {failure.publishFeedback();} catch (const std::runtime_error &) {threw = true;}
  expect(threw);
  for (const auto & wheel : failure.drivers_) expect(wheel.consumed == 0);
  MultiPumaNode aged;
  for (auto & wheel : aged.drivers_) wheel.fill(-1, now_ns()-201000000);
  aged.publishFeedback(); expect(aged.feedback_pub_->writes == 0);
  MultiPumaNode queued;
  for (unsigned f = 0; f < Policy::Count; ++f) queued.frameCallback(can_hardware::Frame{f});
  const auto receipt = queued.recv_msg_queue_.front().ingress_ns;
  std::this_thread::sleep_for(std::chrono::milliseconds(210));
  queued.processQueuedFrames();
  expect(queued.recv_msg_queue_.empty());
  expect(now_ns()-receipt > 200000000);
  expect(queued.queue_age_count_ == Policy::Count);
  expect(queued.queue_age_max_ns_ > 200000000);
  expect(queued.queue_age_over_100ms_ == Policy::Count);
  // A later dequeue closes the five-second diagnostic bucket and resets it.
  queued.queue_timing_started_ns_ = now_ns()-5100000000LL;
  queued.frameCallback(can_hardware::Frame{0});
  queued.processQueuedFrames();
  expect(queued.queue_age_count_ == 0 && queued.queue_age_max_ns_ == 0);
  expect(queued.queue_age_over_100ms_ == 0 && queued.queue_timing_started_ns_ > 0);
  queued.publishFeedback(); expect(queued.feedback_pub_->writes == 0);
  // Fresh callback deliveries recover; old dequeue time was never substituted.
  for (unsigned f = 0; f < Policy::Count; ++f) queued.frameCallback(can_hardware::Frame{f});
  queued.processQueuedFrames(); queued.publishFeedback();
  expect(queued.feedback_pub_->writes == 1);
  std::cout << checks << " aggregate-routine assertions passed\n";
}
'''
    (output/'check.cpp').write_text(program)
    build = subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-O0',
                            '-fsanitize=undefined', '-fno-sanitize-recover=all',
                            str(output/'check.cpp'), '-o', str(output/'check')],
                           capture_output=True, text=True)
    (output/'compiler.log').write_text(build.stdout+build.stderr)
    build.check_returncode()
    result = subprocess.run([str(output/'check')], check=True, capture_output=True, text=True)
    (output/'execution.log').write_text(result.stdout+result.stderr)
    report = {'status': 'offline-aggregate-routines-verified', 'stdout': result.stdout.strip(),
              'node_source_sha256': hashlib.sha256(node_source.encode()).hexdigest(),
              'live_ros_or_can_access': False,
              'limits': 'Actual aggregate/callback/queue-drain bodies, inert publisher/wheel adapters and synthetic frames; no DDS, real receive-thread or physical transport qualification.'}
    (output/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(verify(args.package, args.output), indent=2))
