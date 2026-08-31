"""Exact integer representations of ROS-shaped timestamps, without ROS imports."""


def stamp_key(stamp) -> tuple[int, int]:
    return int(stamp.sec), int(stamp.nanosec)


def stamp_to_nanoseconds(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
