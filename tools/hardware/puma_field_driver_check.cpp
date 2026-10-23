// Offline integration test: real Driver methods, null transport/node handles.
// Never construct a SocketCanDriver or a ROS node, and never send a command.
#include "puma_motor_driver/driver.hpp"
#include <array>
#include <iostream>
#include <stdexcept>
#include <string>

using puma_motor_driver::Driver;
constexpr int64_t ms = 1000000;
constexpr int64_t initial = 1000 * ms;
unsigned checks = 0;
void expect_at(bool value, int line)
{
  if (!value) {
    throw std::runtime_error(
            "offline field/driver integration assertion failed at line " + std::to_string(line));
  }
  ++checks;
}
#define expect(value) expect_at((value), __LINE__)
const std::array<uint32_t, 5> apis{{LM_API_STATUS_VOLTOUT, LM_API_STATUS_CURRENT,
  LM_API_STATUS_POS, LM_API_STATUS_SPD, LM_API_SPD_SET}};
void field(Driver & driver, unsigned index, int64_t stamp, unsigned bytes = 0)
{
  can_hardware::Frame frame;
  frame.id = apis[index] | driver.deviceNumber();
  frame.is_extended = true;
  frame.dlc = bytes ? bytes : (index < 2 ? 2 : 4);
  frame.data.fill(0);
  driver.processMessage(frame, stamp);
}
void fill(Driver & driver, int64_t stamp)
{
  for (unsigned index = 0; index < apis.size(); ++index) field(driver, index, stamp);
}
int main()
{
  Driver driver(nullptr, nullptr, 5, "offline");
  driver.clearMsgCache();
  expect(!driver.feedbackReady(initial));
  fill(driver, initial);
  expect(driver.feedbackReady(initial));
  // Motion fields (position/speed) are bounded at 150 ms.
  expect(driver.feedbackReady(initial + 150*ms));
  expect(!driver.feedbackReady(initial + 150*ms + 1));
  driver.feedbackPublished();
  expect(!driver.feedbackReady(initial));
  field(driver, 2, initial + 50*ms);
  expect(!driver.feedbackReady(initial + 50*ms));
  field(driver, 3, initial + 50*ms);
  expect(driver.feedbackReady(initial + 50*ms));
  diagnostic_updater::DiagnosticStatusWrapper diagnostic;
  driver.runFreqStatus(diagnostic);
  expect(driver.feedbackReady(initial + 50*ms));
  // No wheel can borrow any one of another wheel's five field receipts.
  const std::array<uint8_t, 4> ids{{5, 4, 2, 3}};
  for (unsigned missing = 0; missing < 4; ++missing) {
    for (unsigned absent = 0; absent < apis.size(); ++absent) {
      for (unsigned wheel = 0; wheel < 4; ++wheel) {
        Driver d(nullptr, nullptr, ids[wheel], "offline");
        d.clearMsgCache();
        for (unsigned f = 0; f < apis.size(); ++f)
          if (wheel != missing || f != absent) field(d, f, initial);
        expect(d.feedbackReady(initial) == (wheel != missing));
      }
    }
  }
  // A truncated accepted field invalidates readiness; valid next data recovers.
  for (unsigned f = 0; f < apis.size(); ++f) {
    driver.clearMsgCache(); fill(driver, initial);
    field(driver, f, initial + 1, 1);
    expect(!driver.feedbackReady(initial + 1));
    field(driver, f, initial + 2);
    expect(driver.feedbackReady(initial + 2));
  }
  // Ingress time remains old even if decoding/publication happens much later.
  driver.clearMsgCache(); fill(driver, initial);
  expect(!driver.feedbackReady(initial + 151*ms));
  // Slow auxiliary fields are reused up to 500 ms while motion stays fresh.
  driver.clearMsgCache(); fill(driver, initial);
  for (int64_t t = 50; t <= 550; t += 50) {
    driver.feedbackPublished();
    field(driver, 2, initial + t*ms); field(driver, 3, initial + t*ms);
    expect(driver.feedbackReady(initial + t*ms) == (t <= 500));
  }
  // Fresh auxiliary fields restore readiness only once all three arrive.
  field(driver, 4, initial + 560*ms);
  expect(!driver.feedbackReady(initial + 560*ms));
  field(driver, 0, initial + 560*ms); field(driver, 1, initial + 560*ms);
  expect(driver.feedbackReady(initial + 560*ms));
  // A new speed without a new position withholds publication.
  driver.feedbackPublished();
  field(driver, 3, initial + 600*ms);
  expect(!driver.feedbackReady(initial + 600*ms));
  field(driver, 2, initial + 610*ms);
  expect(driver.feedbackReady(initial + 610*ms));
  // Asymmetric wheels: one with a 400 ms old setpoint, one with all fields
  // fresh, both publish; the slow one stops at the auxiliary bound only.
  Driver slow(nullptr, nullptr, 3, "slow"), quick(nullptr, nullptr, 2, "quick");
  slow.clearMsgCache(); quick.clearMsgCache();
  fill(slow, initial); fill(quick, initial + 400*ms);
  field(slow, 2, initial + 400*ms); field(slow, 3, initial + 400*ms);
  expect(slow.feedbackReady(initial + 400*ms) && quick.feedbackReady(initial + 400*ms));
  // An old duty or current field also bounds readiness at 500 ms.
  for (unsigned aux = 0; aux < 2; ++aux) {
    Driver d(nullptr, nullptr, 5, "aux"); d.clearMsgCache(); fill(d, initial);
    for (unsigned f = 1; f < apis.size(); ++f)
      if (f != aux) field(d, f, initial + 501*ms);
    if (aux == 1) field(d, 0, initial + 501*ms);
    expect(!d.feedbackReady(initial + 501*ms));
  }
  // Other-device frames cannot initialize this wheel's cache metadata.
  Driver other(nullptr, nullptr, 4, "other"); other.clearMsgCache();
  for (unsigned f = 0; f < apis.size(); ++f) {
    can_hardware::Frame frame;
    frame.id = apis[f] | 5; frame.is_extended = true;
    frame.dlc = f < 2 ? 2 : 4; frame.data.fill(0);
    other.processMessage(frame, initial);
  }
  expect(!other.feedbackReady(initial));
  driver.resetConfiguration(); expect(!driver.feedbackReady(initial));
  std::cout << checks << " actual-driver integration assertions passed\n";
}
