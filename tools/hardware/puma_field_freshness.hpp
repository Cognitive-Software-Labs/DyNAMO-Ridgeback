// Offline speed-mode feedback policy candidate. No transport or ROS access.
// The caller must timestamp frames at ingress, before queueing/decode, and
// invalidate a field when its payload is malformed. This policy tracks receipt
// metadata only; it must not be used to certify physical wheel synchronization.
#ifndef DYNAMO_PUMA_FIELD_FRESHNESS_HPP
#define DYNAMO_PUMA_FIELD_FRESHNESS_HPP

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace dynamo
{
class PumaFieldFreshness
{
public:
  enum Field : std::size_t {Duty, Current, Position, Speed, Setpoint, Count};

  explicit PumaFieldFreshness(std::int64_t age_bound_ns)
  : PumaFieldFreshness(age_bound_ns, age_bound_ns) {}

  // Position/speed feed odometry and must be new on every publication; the
  // motion bound limits their age. Duty, current and setpoint are only
  // validated downstream and arrive more slowly and unevenly per wheel (setpoint
  // replies have shown 250 ms gaps while speed stayed near 40 ms), so they
  // get a separate, looser auxiliary bound instead of withholding odometry.
  PumaFieldFreshness(std::int64_t motion_bound_ns, std::int64_t auxiliary_bound_ns)
  : motion_bound_ns_(motion_bound_ns), auxiliary_bound_ns_(auxiliary_bound_ns)
  {
    if (motion_bound_ns <= 0 || auxiliary_bound_ns < motion_bound_ns) {
      throw std::invalid_argument("positive motion bound no greater than the auxiliary bound required");
    }
  }

  // Payload widths match the inspected speed-mode fixed-point decoders.
  // Other control modes need their own setpoint layout and qualification.
  bool receive(Field field, std::int64_t ingress_ns, std::size_t payload_bytes)
  {
    if (field >= Count) {
      return false;
    }
    auto & record = fields_[field];
    constexpr std::array<std::size_t, Count> widths{{2, 2, 4, 4, 4}};
    if (payload_bytes != widths[field] || ingress_ns <= 0 ||
      ingress_ns <= record.ingress_ns)
    {
      // A malformed update must not retain an apparently fresh cache entry.
      record.ingress_ns = std::max(record.ingress_ns, ingress_ns);
      record.valid = false;
      record.updated = false;
      return false;
    }
    record.ingress_ns = ingress_ns;
    record.valid = true;
    record.updated = true;
    return true;
  }

  bool ready(std::int64_t now_ns) const
  {
    const auto motion = motion_age_ns(now_ns);
    const auto auxiliary = auxiliary_age_ns(now_ns);
    return motion >= 0 && motion <= motion_bound_ns_ &&
           auxiliary >= 0 && auxiliary <= auxiliary_bound_ns_ &&
           fields_[Position].updated && fields_[Speed].updated;
  }

  // Call only after ALL wheels pass ready() and the aggregate is published.
  // Slow auxiliary fields may be reused until their explicit age bound.
  void published()
  {
    fields_[Position].updated = false;
    fields_[Speed].updated = false;
  }

  // -1 means unseen/future/invalid data, never a fresh zero-age field.
  std::int64_t oldest_age_ns(std::int64_t now_ns) const
  {
    return age_ns(now_ns, kAll);
  }
  std::int64_t motion_age_ns(std::int64_t now_ns) const
  {
    return age_ns(now_ns, kMotion);
  }
  std::int64_t auxiliary_age_ns(std::int64_t now_ns) const
  {
    return age_ns(now_ns, kAll & ~kMotion);
  }

  void clear() {fields_ = {};}

private:
  static constexpr unsigned kAll = (1u << Count) - 1;
  static constexpr unsigned kMotion = (1u << Position) | (1u << Speed);

  std::int64_t age_ns(std::int64_t now_ns, unsigned mask) const
  {
    if (now_ns <= 0) {
      return -1;
    }
    std::int64_t oldest = 0;
    for (std::size_t field = 0; field < Count; ++field) {
      if (!(mask & (1u << field))) {
        continue;
      }
      const auto & record = fields_[field];
      if (!record.valid || record.ingress_ns <= 0 || record.ingress_ns > now_ns) {
        return -1;
      }
      oldest = std::max(oldest, now_ns - record.ingress_ns);
    }
    return oldest;
  }

  struct Record
  {
    std::int64_t ingress_ns = 0;
    bool valid = false;
    bool updated = false;
  };
  const std::int64_t motion_bound_ns_;
  const std::int64_t auxiliary_bound_ns_;
  std::array<Record, Count> fields_{};
};
}  // namespace dynamo

#endif  // DYNAMO_PUMA_FIELD_FRESHNESS_HPP
