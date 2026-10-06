#!/usr/bin/env python3
"""Read-only camera/web readiness gate using direct DDS discovery, never ros2 CLI.

Source ROS Jazzy and the vendor workspace before execution. Select the deployed
CycloneDDS file explicitly with --profile. Readiness means that each required
vendor image topic has a live Image publisher in this process's graph and an
exact entry in robot_webrtc's HTTP /topics JSON array. It does not prove image
frame delivery, encoder health, or a browser's WebRTC connection.

No images are subscribed, no commands are published, and no services are
restarted. A ROS timer wakes the executor to reevaluate the conditions while a
single monotonic deadline bounds waiting; there is no startup/stability sleep.
"""

import argparse
import json
import math
import os
from pathlib import Path
from queue import Empty, Queue
import re
from threading import Thread
import time
from urllib.error import URLError
from urllib.request import ProxyHandler, build_opener


IMAGE_TYPE = 'sensor_msgs/msg/Image'
VENDOR_STREAMS = (
    'camera/color/image_raw',
    'camera/depth/image_rect_mono8',
    'camera/infra1/image_rect_raw',
)
MAX_HTTP_BYTES = 1024 * 1024


def read_namespace(path):
    """Read the robot-local namespace; YAML constructors are never executed."""
    import yaml

    with Path(path).open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    try:
        namespace = data['system']['ros2']['namespace']
    except (KeyError, TypeError) as error:
        raise ValueError('robot config must contain system.ros2.namespace') from error
    if not isinstance(namespace, str):
        raise ValueError('system.ros2.namespace must be a nonempty ROS namespace')
    namespace = namespace.strip('/')
    if not namespace or any(not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', part)
                            for part in namespace.split('/')):
        raise ValueError('system.ros2.namespace must be a nonempty ROS namespace')
    return namespace


def required_topics(namespace):
    return tuple(f'/{namespace}/{suffix}' for suffix in VENDOR_STREAMS)


def parse_web_topics(payload):
    """The deployed robot_webrtc /topics contract is a flat array of strings."""
    topics = json.loads(payload)
    if not isinstance(topics, list) or any(not isinstance(item, str) for item in topics):
        raise ValueError('/topics response must be a JSON array of topic strings')
    return set(topics)


def evaluate_readiness(required, publisher_types, web_topics, web_error=None):
    """Publisher endpoint types, not advertised names/subscriptions, are evidence."""
    missing_publishers = [topic for topic in required
                          if IMAGE_TYPE not in publisher_types.get(topic, ())]
    missing_web_topics = [topic for topic in required if topic not in web_topics]
    return {
        'ready': not missing_publishers and not missing_web_topics and web_error is None,
        'missing_publishers': missing_publishers,
        'missing_web_topics': missing_web_topics,
        'web_error': web_error,
    }


def wait_for_readiness(sample, wait, timeout, interval, emit, clock=time.monotonic):
    """Dependency-injected loop: wait processes events, sample receives HTTP budget.

    Waiting is bounded by the same deadline as observation. Readiness observed
    after that deadline cannot turn a timed-out startup into a success.
    """
    for name, value in (('timeout', timeout), ('interval', interval)):
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be finite and positive')
    start = clock()
    deadline = start + timeout
    previous = None
    state = {'ready': False, 'missing_publishers': [], 'missing_web_topics': [],
             'web_error': 'no observation before deadline'}
    while clock() < deadline:
        state = sample(deadline - clock())
        now = clock()
        if now >= deadline:
            break
        if state['ready']:
            result = {**state, 'status': 'ready', 'elapsed_s': round(now - start, 3)}
            emit(result)
            return result
        key = (tuple(state['missing_publishers']), tuple(state['missing_web_topics']),
               state['web_error'])
        if key != previous:
            emit({**state, 'status': 'waiting', 'elapsed_s': round(now - start, 3)})
            previous = key
        remaining = deadline - clock()
        if remaining > 0:
            wait(min(interval, remaining))
    result = {**state, 'ready': False, 'status': 'timeout',
              'elapsed_s': round(clock() - start, 3)}
    emit(result)
    return result


def _fetch_web_topics(url, timeout):
    # Bypass ambient proxy settings: this is the local vendor endpoint. Limit
    # reads so a malformed response cannot consume unbounded memory.
    opener = build_opener(ProxyHandler({}))
    with opener.open(url, timeout=timeout) as response:
        payload = response.read(MAX_HTTP_BYTES + 1)
    if len(payload) > MAX_HTTP_BYTES:
        raise ValueError('/topics response exceeds 1 MiB')
    return parse_web_topics(payload)


def fetch_web_topics(url, timeout):
    """Bound the entire GET, including a peer trickling bytes between timeouts.

    urllib's socket timeout alone only bounds individual reads. A daemon worker
    lets the observer stop at its monotonic deadline even if a local HTTP peer
    never finishes its response. No worker is joined at process shutdown.
    """
    result = Queue(maxsize=1)

    def request():
        try:
            value = _fetch_web_topics(url, timeout)
        except Exception as error:
            result.put((None, error))
        else:
            result.put((value, None))

    started = time.monotonic()
    Thread(target=request, daemon=True).start()
    try:
        value, error = result.get(timeout=max(0, timeout - (time.monotonic() - started)))
    except Empty as error:
        raise TimeoutError('/topics GET exceeded its observation budget') from error
    if error is not None:
        raise error
    return value


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--domain', type=int, required=True)
    result.add_argument('--profile', type=Path, required=True)
    result.add_argument('--robot-config', type=Path, default=Path('/etc/clearpath/robot.yaml'))
    result.add_argument('--topics-url', default='http://127.0.0.1:9003/topics')
    result.add_argument('--timeout', type=float, default=90.0)
    result.add_argument('--interval', type=float, default=0.5)
    result.add_argument('--http-timeout', type=float, default=1.0)
    return result


def run(args):
    """Only this entry point imports ROS; pure tests need no DDS participant."""
    metadata = {'domain': args.domain, 'robot_config': str(args.robot_config),
                'profile': str(args.profile), 'topics_url': args.topics_url,
                'discovery': 'direct_rclpy_publisher_endpoints'}

    def emit(state):
        print(json.dumps({'event': 'camera_web_readiness', **metadata, **state},
                         sort_keys=True), flush=True)

    context = node = executor = None
    try:
        if not 0 <= args.domain <= 232:
            raise ValueError('domain must be in the supported ROS domain range 0..232')
        for name in ('timeout', 'interval', 'http_timeout'):
            if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
                raise ValueError(f'{name} must be finite and positive')
        namespace = read_namespace(args.robot_config)
        required = required_topics(namespace)
        profile = args.profile.resolve(strict=True)
        with profile.open('rb') as stream:
            if not stream.read(1):
                raise ValueError('CycloneDDS profile is empty')
        metadata.update(namespace=namespace, required_topics=required, profile=str(profile))
        os.environ['ROS_DOMAIN_ID'] = str(args.domain)
        os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
        os.environ['CYCLONEDDS_URI'] = profile.as_uri()

        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node

        context = Context()
        rclpy.init(args=[], context=context, domain_id=args.domain)
        node = Node('camera_web_readiness', context=context,
                    enable_rosout=False, start_parameter_services=False,
                    use_global_arguments=False)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        # Jazzy rclpy exposes no public graph-change waiter. Timer events let the
        # executor recheck discovery/HTTP without sleeping or subscribing images.
        node.create_timer(args.interval, lambda: None)

        def sample(remaining):
            web_error = None
            try:
                web_topics = fetch_web_topics(
                    args.topics_url, min(args.http_timeout, remaining))
            except (OSError, URLError, ValueError, UnicodeError) as error:
                web_topics = set()
                web_error = f'{type(error).__name__}: {error}'
            # Query endpoints after HTTP so an earlier, stale publisher snapshot
            # cannot satisfy readiness if the publisher vanished during the GET.
            publishers = {
                topic: [endpoint.topic_type for endpoint in
                        node.get_publishers_info_by_topic(topic)] for topic in required
            }
            return evaluate_readiness(required, publishers, web_topics, web_error)

        state = wait_for_readiness(
            sample, lambda seconds: executor.spin_once(timeout_sec=seconds),
            args.timeout, args.interval, emit)
        return 0 if state['ready'] else 1
    except KeyboardInterrupt:
        emit({'ready': False, 'status': 'interrupted'})
        return 130
    except Exception as error:
        emit({'ready': False, 'status': 'error',
              'error': f'{type(error).__name__}: {error}'})
        return 2
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=0)
        if node is not None:
            node.destroy_node()
        if context is not None and context.ok():
            context.shutdown()


if __name__ == '__main__':
    raise SystemExit(run(parser().parse_args()))
