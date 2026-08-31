"""Ground-truth display contract shared with the benchmark publisher."""

from __future__ import annotations

from dataclasses import dataclass

from geometry_msgs.msg import PointStamped

from ridgeback_autonomy.common.stamps import stamp_to_nanoseconds

# How long a truth message stays displayable. The runner publishes only while a
# capture window is open, so the gate is what makes the line disappear between
# trials instead of sitting next to a target that has already been teleported
# away. Defined here rather than in each consumer: the viz node and the overlay
# have to expire on the same schedule or they disagree about the same message.
TRUTH_MAX_AGE_S = 3.0


@dataclass(frozen=True)
class TruthReading:
    """One trial's ground truth as the display surfaces consume it.

    ``trial_id`` names the trial the numbers belong to, so a reading can be
    checked against the scene on screen instead of being taken on trust.
    """

    lateral_m: float
    forward_m: float
    distance_m: float
    trial_id: str


def ground_truth_point_message(
    true_pose: dict[str, float],
    trial_id: str,
) -> PointStamped:
    """Pack x=lateral, y=forward, z=distance for the display contract."""

    msg = PointStamped()
    msg.header.frame_id = trial_id
    msg.point.x = float(true_pose['lateral_m'])
    msg.point.y = float(true_pose['forward_m'])
    msg.point.z = float(true_pose['distance_m'])
    return msg


def truth_reading(msg, now_nanoseconds: int, max_age_s: float = TRUTH_MAX_AGE_S):
    """Unpack a truth message, or ``None`` once it is too old to display.

    Age is measured on the message STAMP, never on when it arrived. A message
    delayed behind a full subscription queue is stale data however recently it
    was handed to the callback, and a receipt-time gate cannot tell those two
    apart -- it reports the delay as freshness and pins the previous trial's
    truth under the current trial's scene.

    Packed by the runner as x=lateral, y=forward, z=distance: the base-frame
    planar measurement every benchmark row shares, not a 3D point in any TF
    frame. The trial id rides in ``header.frame_id`` for the same reason.
    """

    if msg is None:
        return None
    stamp_nanoseconds = stamp_to_nanoseconds(msg.header.stamp)
    if now_nanoseconds - stamp_nanoseconds > max_age_s * 1_000_000_000:
        return None
    return TruthReading(
        lateral_m=float(msg.point.x),
        forward_m=float(msg.point.y),
        distance_m=float(msg.point.z),
        trial_id=str(msg.header.frame_id),
    )
