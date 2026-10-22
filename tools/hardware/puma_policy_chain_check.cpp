// Offline policy-chain check. No ROS node, CAN device, or motor command.
#include "puma_field_freshness.hpp"
#include "puma_feedback_safety.hpp"

#include <array>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>

using dynamo::PumaFieldFreshness;
using clearpath_hardware_interfaces::FeedbackSafety;
using clearpath_hardware_interfaces::FeedbackWheelSample;

constexpr std::int64_t ms = 1000000;
constexpr std::int64_t start = 1000 * ms;
using Wheels = std::array<PumaFieldFreshness, 4>;

void expect(bool condition)
{
  if (!condition) throw std::runtime_error("field-to-consumer policy assertion failed");
}

std::vector<FeedbackWheelSample> complete_batch()
{
  return {{"fl", 5, 0, 0, 0, 0, 0}, {"fr", 4, 0, 0, 0, 0, 0},
          {"rl", 2, 0, 0, 0, 0, 0}, {"rr", 3, 0, 0, 0, 0, 0}};
}

bool ready(const Wheels & wheels, std::int64_t now)
{
  for (const auto & wheel : wheels) if (!wheel.ready(now)) return false;
  return true;
}

void published(Wheels & wheels)
{
  for (auto & wheel : wheels) wheel.published();
}

void receive(Wheels & wheels, unsigned field, std::int64_t at)
{
  for (auto & wheel : wheels) {
    expect(wheel.receive(static_cast<PumaFieldFreshness::Field>(field),
                         at, field < 2 ? 2 : 4));
  }
}

int main()
{
  Wheels wheels{{PumaFieldFreshness(200 * ms), PumaFieldFreshness(200 * ms),
                 PumaFieldFreshness(200 * ms), PumaFieldFreshness(200 * ms)}};
  FeedbackSafety consumer({"fl", "fr", "rl", "rr"}, .25);
  for (unsigned field = 0; field < PumaFieldFreshness::Count; ++field)
    receive(wheels, field, start);
  expect(ready(wheels, start));
  expect(consumer.activate(complete_batch(), 1.0, 1.0));
  published(wheels);

  // One slow auxiliary setpoint with a 300 ms inter-arrival gap blocks only
  // one 50 ms aggregate tick at the experimental 200 ms field-age limit.
  // The 250 ms consumer timeout must not be reached by that 100 ms gap.
  double previous_receipt = 1.0;
  unsigned withheld = 0;
  for (int tick = 1; tick <= 8; ++tick) {
    const auto at = start + tick * 50 * ms;
    for (unsigned field = 0; field < 4; ++field) receive(wheels, field, at);
    if (tick >= 6) receive(wheels, 4, at);
    const double now = 1.0 + tick * .05;
    if (ready(wheels, at)) {
      expect(now - previous_receipt <= .25);
      expect(consumer.accept(complete_batch(), now, now));
      previous_receipt = now;
      published(wheels);
    } else {
      ++withheld;
    }
    expect(consumer.safe(now));
  }
  expect(withheld == 1);

  // A missing speed field for one wheel suppresses every aggregate. After
  // more than 250 ms without an aggregate, the consumer latches a fault.
  for (int tick = 9; tick <= 14; ++tick) {
    const auto at = start + tick * 50 * ms;
    for (unsigned field = 0; field < PumaFieldFreshness::Count; ++field) {
      if (field == PumaFieldFreshness::Speed) {
        for (unsigned wheel = 0; wheel < wheels.size() - 1; ++wheel)
          expect(wheels[wheel].receive(PumaFieldFreshness::Speed, at, 4));
      } else {
        receive(wheels, field, at);
      }
    }
    expect(!ready(wheels, at));
  }
  expect(!consumer.safe(1.70));
  expect(consumer.faulted());
  expect(wheels.back().receive(PumaFieldFreshness::Speed, start + 701 * ms, 4));
  expect(ready(wheels, start + 701 * ms));
  expect(!consumer.accept(complete_batch(), 1.701, 1.701));
  // Only supervised reactivation with a complete fresh batch clears it.
  expect(consumer.activate(complete_batch(), 1.701, 1.701));
  expect(consumer.safe(1.701));

  std::cout << "policy chain passed: one withheld tick, consumer fault on long wheel gap, explicit recovery\n";
}
