#!/usr/bin/env python3
"""Camera-sized DDS traffic probe for the Intel–Thor Ethernet link.

``pub`` publishes synthetic D455-like frames at a fixed rate: a colour stream
(rgb8) and optionally an aligned-depth stream (16UC1) with the same sequence
number and stamp per tick. ``sub`` receives them for a fixed window and prints
one JSON line. The primary keys used by ``check_link`` are ``received``,
``rate_hz``, ``max_gap_s`` and ``udp_drops`` (kernel receive-buffer drops on
the probe's own sockets). The additional keys are:

- ``loss_pct``: colour frames missing from the sequence range seen.
- ``paired_pct``: colour frames whose same-sequence depth frame also arrived,
  the exact-stamp contract of the mask measurement node.
- ``cpu_pct``: process CPU time as a percentage of one core.

With ``--echo`` the subscriber returns a small acknowledgement for every colour
frame, and the publisher reports the round-trip time. That time is the colour
frame's one-way transfer plus a small reply, so it needs no clock
synchronisation between the hosts.

Run both ends in an otherwise unused ROS domain (``ROS_DOMAIN_ID``) so the
probe never mixes with the robot graph. ``check_link`` and ``rmw_benchmark``
in this directory drive it.
"""

import argparse
import json
import os
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

COLOR_TOPIC = 'dds_link_probe/image'
DEPTH_TOPIC = 'dds_link_probe/depth'
ACK_TOPIC = 'dds_link_probe/ack'
QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
ACK_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
)


def own_udp_drops():
    """Kernel receive-buffer drops summed over this process's UDP sockets."""
    inodes = set()
    for fd in os.listdir('/proc/self/fd'):
        try:
            target = os.readlink(f'/proc/self/fd/{fd}')
        except OSError:
            continue
        if target.startswith('socket:['):
            inodes.add(target[8:-1])
    drops = 0
    for table in ('/proc/net/udp', '/proc/net/udp6'):
        with open(table, encoding='ascii') as stream:
            next(stream)
            for line in stream:
                fields = line.split()
                if fields[9] in inodes:
                    drops += int(fields[-1])
    return drops


class CpuMeter:
    """Process CPU time as a percentage of one core since construction."""

    def __init__(self):
        self._cpu = time.process_time()
        self._wall = time.monotonic()

    def percent(self):
        wall = time.monotonic() - self._wall
        return round(100.0 * (time.process_time() - self._cpu) / wall, 1) if wall else None


def make_image(width, height, encoding):
    channels = {'rgb8': 3, '16UC1': 2}[encoding]
    image = Image(height=height, width=width, encoding=encoding, step=width * channels)
    image.data = bytes(width * height * channels)
    return image


def publish(args):
    node = Node('dds_link_probe_pub')
    color_pub = node.create_publisher(Image, COLOR_TOPIC, QOS)
    depth_pub = node.create_publisher(Image, DEPTH_TOPIC, QOS) if args.depth else None
    color = make_image(args.width, args.height, 'rgb8')
    depth = make_image(args.width, args.height, '16UC1') if args.depth else None

    sent_at = {}
    round_trips = []

    def on_ack(msg):
        sent = sent_at.pop(int(msg.frame_id), None)
        if sent is not None:
            round_trips.append(time.monotonic() - sent)

    if args.echo:
        node.create_subscription(Header, ACK_TOPIC, on_ack, ACK_QOS)

    cpu = CpuMeter()
    period = 1.0 / args.rate
    deadline = time.monotonic() + args.duration
    next_send = time.monotonic()
    sequence = 0
    while rclpy.ok() and time.monotonic() < deadline:
        stamp = node.get_clock().now().to_msg()
        for image, publisher in ((color, color_pub), (depth, depth_pub)):
            if publisher is not None:
                image.header.stamp = stamp
                image.header.frame_id = str(sequence)
                if image is color:
                    sent_at[sequence] = time.monotonic()
                publisher.publish(image)
        sequence += 1
        next_send += period
        # Service acknowledgements while waiting for the next tick.
        while True:
            remaining = next_send - time.monotonic()
            if remaining <= 0:
                break
            rclpy.spin_once(node, timeout_sec=remaining if args.echo else 0.0)
            if not args.echo:
                time.sleep(max(0.0, next_send - time.monotonic()))
                break

    result = {'sent': sequence, 'cpu_pct': cpu.percent()}
    if args.echo:
        ms = sorted(1000.0 * value for value in round_trips)
        result['acked'] = len(ms)
        if ms:
            result['rtt_ms_p50'] = round(statistics.median(ms), 2)
            result['rtt_ms_p95'] = round(ms[int(0.95 * (len(ms) - 1))], 2)
            result['rtt_ms_max'] = round(ms[-1], 2)
    print(json.dumps(result))
    node.destroy_node()


def subscribe(args):
    node = Node('dds_link_probe_sub')
    arrivals = []
    color_sequences = []
    depth_sequences = set()
    ack_pub = node.create_publisher(Header, ACK_TOPIC, ACK_QOS) if args.echo else None

    def on_color(msg):
        arrivals.append(time.monotonic())
        color_sequences.append(int(msg.header.frame_id))
        if ack_pub is not None:
            ack_pub.publish(Header(frame_id=msg.header.frame_id))

    node.create_subscription(Image, COLOR_TOPIC, on_color, QOS)
    if args.depth:
        node.create_subscription(
            Image, DEPTH_TOPIC,
            lambda msg: depth_sequences.add(int(msg.header.frame_id)), QOS)

    # Discovery and publisher start-up are excluded from the measured window.
    first_deadline = time.monotonic() + args.wait
    while rclpy.ok() and not arrivals and time.monotonic() < first_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    arrivals.clear()
    color_sequences.clear()
    depth_sequences.clear()
    drops_before = own_udp_drops()
    cpu = CpuMeter()
    start = time.monotonic()
    while rclpy.ok() and time.monotonic() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)

    gaps = [b - a for a, b in zip(arrivals, arrivals[1:])]
    result = {
        'received': len(arrivals),
        'rate_hz': round(len(arrivals) / args.duration, 2),
        'max_gap_s': round(max(gaps), 3) if gaps else None,
        'udp_drops': own_udp_drops() - drops_before,
        'cpu_pct': cpu.percent(),
    }
    if color_sequences:
        expected = max(color_sequences) - min(color_sequences) + 1
        result['loss_pct'] = round(100.0 * (1 - len(set(color_sequences)) / expected), 2)
        # The window edges can cut a colour/depth pair in half; judge the interior.
        interior = {s for s in color_sequences
                    if min(color_sequences) < s < max(color_sequences)}
        if args.depth and interior:
            paired = len(interior & depth_sequences)
            result['paired_pct'] = round(100.0 * paired / len(interior), 2)
    print(json.dumps(result))
    node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('mode', choices=('pub', 'sub'))
    parser.add_argument('--rate', type=float, default=30.0)
    parser.add_argument('--duration', type=float, default=15.0)
    parser.add_argument('--wait', type=float, default=15.0,
                        help='sub: seconds to wait for the first frame')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--depth', action='store_true',
                        help='also stream same-sequence 16UC1 depth frames')
    parser.add_argument('--echo', action='store_true',
                        help='acknowledge colour frames for round-trip timing')
    args = parser.parse_args()
    rclpy.init()
    try:
        if args.mode == 'pub':
            publish(args)
        else:
            subscribe(args)
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
