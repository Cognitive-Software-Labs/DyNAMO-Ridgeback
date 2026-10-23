// Maintained source: tools/hardware/puma_feedback_safety.hpp
// Policy used by the installed candidate; not a certified safety component.
#ifndef DYNAMO_PUMA_FEEDBACK_SAFETY_HPP
#define DYNAMO_PUMA_FEEDBACK_SAFETY_HPP
#include <cmath>
#include <set>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace clearpath_hardware_interfaces {
struct FeedbackWheelSample {
  std::string name;
  unsigned device;
  double speed, travel, duty_cycle, current, setpoint;
};

class FeedbackSafety {
public:
  FeedbackSafety(std::vector<std::string> expected, double timeout = .25)
  : expected_(expected.begin(), expected.end()), timeout_(timeout) {
    if (expected.empty() || expected_.size() != expected.size() ||
        !std::isfinite(timeout) || timeout <= 0 || timeout > 1.)
      throw std::invalid_argument("invalid feedback identities/timeout");
  }
  bool activate(const std::vector<FeedbackWheelSample>& batch, double receipt, double now) {
    if (!valid(batch) || !valid_age(receipt, now)) { fail(); return false; }
    devices_.clear();
    for (const auto& wheel : batch) devices_[wheel.name] = wheel.device;
    active_=true; fault_=false; receipt_=receipt; return true;
  }
  bool accept(const std::vector<FeedbackWheelSample>& batch, double receipt, double now) {
    if (!active_ || fault_) return false;
    if (!valid(batch) || !valid_age(receipt,now) || receipt < receipt_) {
      fail(); return false;
    }
    for (const auto& wheel : batch) {
      if (devices_.at(wheel.name) != wheel.device) { fail(); return false; }
    }
    // New feedback cannot erase an already expired receive interval.
    if (!valid_age(receipt_,now)) { fail(); return false; }
    receipt_=receipt; return true;
  }
  bool safe(double now) {
    if (!active_ || fault_) return false;
    if (!valid_age(receipt_,now)) { fail(); return false; }
    return true;
  }
  void fail() { fault_=true; }
  void deactivate() { active_=false; fault_=true; }
  bool faulted() const { return fault_; }
private:
  bool valid_age(double receipt,double now) const {
    return std::isfinite(receipt) && std::isfinite(now) && now >= receipt && now-receipt <= timeout_;
  }
  bool valid(const std::vector<FeedbackWheelSample>& batch) const {
    if(batch.size()!=expected_.size()) return false;
    std::set<std::string> names;std::set<unsigned> ids;
    for(const auto& wheel:batch) {
      if(!expected_.count(wheel.name) || !names.insert(wheel.name).second ||
         wheel.device==0 || wheel.device>63 || !ids.insert(wheel.device).second ||
         !std::isfinite(wheel.speed) || !std::isfinite(wheel.travel) ||
         !std::isfinite(wheel.duty_cycle) || !std::isfinite(wheel.current) ||
         !std::isfinite(wheel.setpoint)) return false;
    }
    return true;
  }
  std::set<std::string> expected_;
  std::map<std::string,unsigned> devices_;
  double timeout_, receipt_=0;
  bool active_=false, fault_=true;
};
}
#endif
