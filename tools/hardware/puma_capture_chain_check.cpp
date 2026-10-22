// Replay passive CAN receipt times through the maintained producer and consumer
// policies. This has no ROS, CAN interface, or motor command output.
#include "puma_field_freshness.hpp"
#include "puma_feedback_safety.hpp"

#include <array>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

using dynamo::PumaFieldFreshness;
using clearpath_hardware_interfaces::FeedbackSafety;
using clearpath_hardware_interfaces::FeedbackWheelSample;
constexpr std::int64_t ms = 1000000;
struct Event {std::int64_t ns; unsigned wheel; unsigned field; bool valid;};
using Wheels = std::array<PumaFieldFreshness, 4>;

int main(int argc, char ** argv)
{
  // bound_ms phase_ms queue_delay_ms [auxiliary_bound_ms]; one bound applies to all fields.
  if (argc != 4 && argc != 5) return 2;
  const auto bound_ms = std::stoll(argv[1]);
  const auto phase_ms = std::stoll(argv[2]);
  const auto queue_delay_ms = std::stoll(argv[3]);
  const auto aux_ms = argc == 5 ? std::stoll(argv[4]) : bound_ms;
  if (bound_ms <= 0 || aux_ms < bound_ms || phase_ms < 0 || phase_ms >= 50 ||
      queue_delay_ms < 0 || queue_delay_ms > 500) return 2;
  std::vector<Event> events;
  Event event;
  while (std::cin >> event.ns >> event.wheel >> event.field >> event.valid) {
    if (event.wheel >= 4 || event.field >= PumaFieldFreshness::Count ||
        (!events.empty() && event.ns < events.back().ns)) return 3;
    events.push_back(event);
  }
  if (events.empty()) return 4;
  const PumaFieldFreshness policy(bound_ms*ms, aux_ms*ms);
  Wheels wheels{{policy, policy, policy, policy}};
  FeedbackSafety consumer({"fl", "fr", "rl", "rr"}, .25);
  const std::vector<FeedbackWheelSample> batch{{"fl",5,0,0,0,0,0},
    {"fr",4,0,0,0,0,0}, {"rl",2,0,0,0,0,0}, {"rr",3,0,0,0,0,0}};
  const auto widths = std::array<std::size_t, 5>{{2,2,4,4,4}};
  std::size_t index = 0;
  std::int64_t previous_publish = 0, max_gap = 0, max_age = 0;
  unsigned publications = 0, withheld = 0, consumer_faults = 0;
  bool active = false;
  const auto first = events.front().ns;
  const auto last = events.back().ns;
  for (auto tick = first + phase_ms*ms; tick <= last; tick += 50*ms) {
    while (index < events.size() && events[index].ns + queue_delay_ms*ms <= tick) {
      const auto & e = events[index++];
      wheels[e.wheel].receive(static_cast<PumaFieldFreshness::Field>(e.field),
                              e.ns, e.valid ? widths[e.field] : 0);
    }
    bool ready = true;
    for (const auto & wheel : wheels) ready = ready && wheel.ready(tick);
    const double now = static_cast<double>(tick) / 1e9;
    if (ready) {
      for (const auto & wheel : wheels) {
        const auto age = wheel.oldest_age_ns(tick);
        if (wheel.motion_age_ns(tick) > bound_ms*ms || age < 0 || age > aux_ms*ms) return 5;
        if (age > max_age) max_age = age;
      }
      if (active) {
        if (!consumer.accept(batch, now, now)) ++consumer_faults;
      } else if (!consumer.faulted() || publications == 0) {
        if (!consumer.activate(batch, now, now)) return 6;
        active = true;
      }
      if (previous_publish && tick - previous_publish > max_gap)
        max_gap = tick - previous_publish;
      previous_publish = tick;
      ++publications;
      for (auto & wheel : wheels) wheel.published();
    } else {
      ++withheld;
      if (active && !consumer.safe(now)) ++consumer_faults;
    }
  }
  std::cout << std::fixed << std::setprecision(3)
    << "{\"bound_ms\":" << bound_ms << ",\"auxiliary_bound_ms\":" << aux_ms
    << ",\"phase_ms\":" << phase_ms
    << ",\"queue_delay_ms\":" << queue_delay_ms
    << ",\"publications\":" << publications << ",\"withheld_ticks\":" << withheld
    << ",\"max_gap_ms\":" << static_cast<double>(max_gap)/ms
    << ",\"max_field_age_ms\":" << static_cast<double>(max_age)/ms
    << ",\"consumer_fault_events\":" << consumer_faults
    << ",\"consumer_activated\":" << (active ? "true":"false")
    << ",\"consumer_faulted_at_end\":" << (active && consumer.faulted() ? "true":"false")
    << "}\n";
}
