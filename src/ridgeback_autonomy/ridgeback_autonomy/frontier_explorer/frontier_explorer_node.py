#!/usr/bin/env python3

import time as _time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from tf2_ros import TransformListener, Buffer

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from explore_lite_msgs.msg import ExploreStatus
from ridgeback_autonomy.frontier_explorer.navigator import get_frontier_clusters
from ridgeback_autonomy.frontier_explorer.params import UNKNOWN, OBSTACLE, FREE


class FrontierExplorerNode(Node):
    def __init__(self):
        super().__init__('frontier_explorer_node')
        
        # Declare parameters
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('costmap_topic', 'global_costmap/costmap')
        self.declare_parameter('planner_frequency', 0.2)
        self.declare_parameter('progress_timeout', 30.0)
        self.declare_parameter('distance_weight', 0.5)
        self.declare_parameter('size_weight', 0.5)
        self.declare_parameter('min_frontier_size', 6)
        self.declare_parameter('visualize', True)
        self.declare_parameter('lethal_cost_threshold', 80)
        self.declare_parameter('goal_safety_margin', 3)
        self.declare_parameter('goal_cost_threshold', 65)
        self.declare_parameter('goal_advance_cells', 10)
        self.declare_parameter('near_frontier_radius', 90)
        self.declare_parameter('startup_delay', 15.0)
        self.declare_parameter('abort_retry_delay', 5.0)
        self.declare_parameter('abort_blacklist_threshold', 3)
        self.declare_parameter('visited_goal_radius', 25)
        self.declare_parameter('max_effective_frontier_size', 80)
        
        # Get parameters
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.planner_frequency = self.get_parameter('planner_frequency').value
        self.progress_timeout = self.get_parameter('progress_timeout').value
        self.distance_weight = self.get_parameter('distance_weight').value
        self.size_weight = self.get_parameter('size_weight').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.visualize = self.get_parameter('visualize').value
        self.lethal_cost_threshold = self.get_parameter('lethal_cost_threshold').value
        self.goal_safety_margin = self.get_parameter('goal_safety_margin').value
        self._goal_cost_threshold = self.get_parameter('goal_cost_threshold').value
        self._goal_advance_cells = self.get_parameter('goal_advance_cells').value
        self._near_frontier_radius = self.get_parameter('near_frontier_radius').value
        self._startup_delay = self.get_parameter('startup_delay').value
        self._abort_retry_delay = self.get_parameter('abort_retry_delay').value
        self._abort_blacklist_threshold = self.get_parameter('abort_blacklist_threshold').value
        self._visited_goal_radius = self.get_parameter('visited_goal_radius').value
        self._max_effective_frontier_size = self.get_parameter('max_effective_frontier_size').value
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create callback groups for concurrent operations
        callback_group_1 = MutuallyExclusiveCallbackGroup()
        callback_group_2 = MutuallyExclusiveCallbackGroup()
        
        # Publisher for frontier visualization
        self._frontier_pub = self.create_publisher(
            MarkerArray, 'explore/frontiers', 10,
            callback_group=callback_group_2)

        # Exploration status — same topic/QoS contract as explore_lite so
        # benchmark tooling can detect completion regardless of explorer
        status_qos = QoSProfile(depth=10)
        status_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self._status_pub = self.create_publisher(
            ExploreStatus, 'explore/status', status_qos,
            callback_group=callback_group_2)
        self._publish_status(ExploreStatus.EXPLORATION_STARTED)

        # Subscriber for costmap
        self.costmap_subscription = self.create_subscription(
            OccupancyGrid,
            self.costmap_topic,
            self.costmap_callback,
            10,
            callback_group=callback_group_1
        )
        
        # Action client for Nav2 NavigateToPose
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Timer for exploration loop
        self.exploration_timer = self.create_timer(
            1.0 / self.planner_frequency,
            self.exploration_callback,
            callback_group=callback_group_2
        )
        
        # State variables
        self.robot_awareness_map = None
        self.current_costmap = None
        self.costmap_costs = None
        self.robot_position = None
        self.current_goal = None
        self.goal_start_time = None
        self._current_goal_handle = None
        # Startup and retry timing (wall clock so it's immune to sim-time jumps)
        self._start_wall_time = _time.time()
        self._retry_after_wall_time = 0.0
        # Blacklist: list of (grid_x, grid_y) of recently-reached goals to avoid re-picking
        self._visited_goals = []
        # Per-goal failure tallies: list of [(grid_x, grid_y), count]. Radius-
        # matched, not dict-keyed — the re-picked goal for the same frontier
        # drifts a few cells between attempts and must accumulate onto one tally.
        self._abort_counts = []
        # Best distance_remaining seen for the active goal (progress tracking)
        self._goal_best_distance = None
        
        self.get_logger().info(f'Frontier Explorer Node initialized')
        self.get_logger().info(f'  Robot Base Frame: {self.robot_base_frame}')
        self.get_logger().info(f'  Costmap Topic: {self.costmap_topic}')
        self.get_logger().info(f'  Planner Frequency: {self.planner_frequency} Hz')
        
    def costmap_callback(self, msg: OccupancyGrid):
        """Receive and process occupancy grid (costmap)."""
        self.current_costmap = msg
        try:
            self.update_awareness_map(msg)
        except Exception as e:
            self.get_logger().warn(f'Error updating awareness map: {e}')

    def update_awareness_map(self, costmap_msg: OccupancyGrid):
        """Convert occupancy grid to robot awareness map (vectorized).

        Nav2 publishes costmap as OccupancyGrid with values:
          -1 = unknown (NO_INFORMATION)
           0 = free
          1-98 = inflated cost (proximity to obstacles)
          99-100 = lethal/inscribed obstacle

        Builds into a fresh array and swaps the reference at the end: the
        exploration timer reads robot_awareness_map from another executor
        thread, and mutating the shared array in place would let it observe
        a half-written (all-UNKNOWN) map.
        """
        height = costmap_msg.info.height
        width = costmap_msg.info.width
        data = np.array(costmap_msg.data, dtype=np.int8).reshape((height, width))

        awareness = np.full((height, width), UNKNOWN, dtype=np.int8)
        awareness[data == 0] = FREE
        # Inflated cells below threshold are navigable — mark as FREE
        awareness[(data > 0) & (data < self.lethal_cost_threshold)] = FREE
        # High-cost and lethal cells are obstacles
        awareness[data >= self.lethal_cost_threshold] = OBSTACLE
        # Unknown stays UNKNOWN (data == -1)

        # Reference swaps are atomic in CPython — readers see old or new, never mixed
        self.robot_awareness_map = awareness
        # Store raw costmap costs for frontier goal validation
        self.costmap_costs = data
    
    def get_robot_position(self) -> tuple:
        """Get robot position in map frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.current_costmap.header.frame_id,
                self.robot_base_frame,
                rclpy.time.Time()
            )
            
            position = transform.transform.translation
            
            # Convert world position to grid coordinates
            origin_x = self.current_costmap.info.origin.position.x
            origin_y = self.current_costmap.info.origin.position.y
            resolution = self.current_costmap.info.resolution
            
            grid_x = int((position.x - origin_x) / resolution)
            grid_y = int((position.y - origin_y) / resolution)
            
            # Clamp to map bounds
            grid_x = max(0, min(grid_x, self.current_costmap.info.width - 1))
            grid_y = max(0, min(grid_y, self.current_costmap.info.height - 1))
            
            return (grid_x, grid_y)
        except Exception as e:
            self.get_logger().warn(f'Failed to get robot position: {e}')
            return (self.current_costmap.info.width // 2, self.current_costmap.info.height // 2)
    
    def exploration_callback(self):
        """Main exploration loop."""
        if self.current_costmap is None or self.robot_awareness_map is None:
            return
        
        now_wall = _time.time()
        # Wait for TF and Nav2 to stabilize before sending the first goal
        elapsed_since_start = now_wall - self._start_wall_time
        if elapsed_since_start < self._startup_delay:
            return
        # Back off after a quick abort to avoid hammering Nav2 during TF instability
        if now_wall < self._retry_after_wall_time:
            return
        
        try:
            # Get current robot position
            robot_pos = self.get_robot_position()
            
            # Check if we need to select a new goal
            if self.current_goal is None:
                self.select_new_frontier_goal(robot_pos)
            else:
                # Check if goal is still reachable or if we've been stuck
                self.check_goal_progress()
        
        except Exception as e:
            self.get_logger().error(f'Error in exploration loop: {e}')
    
    def is_goal_safe(self, grid_pos):
        """Check if a goal position has enough clearance from obstacles.
        
        Checks a square region around the goal to ensure no high-cost cells
        are within the safety margin (approximating robot footprint).
        """
        if self.costmap_costs is None:
            return True
        
        gx, gy = grid_pos
        margin = self.goal_safety_margin
        h, w = self.costmap_costs.shape
        
        # Extract the region around the goal
        y_min = max(0, gy - margin)
        y_max = min(h, gy + margin + 1)
        x_min = max(0, gx - margin)
        x_max = min(w, gx + margin + 1)
        
        region = self.costmap_costs[y_min:y_max, x_min:x_max]

        # Use goal_cost_threshold (lower than lethal) so goals are placed well
        # inside the navigable corridor, not in heavily-inflated zones near walls.
        if np.any(region >= self._goal_cost_threshold):
            return False
        return True

    def _is_blacklisted(self, pos):
        """Return True if pos is too close to a recently-visited goal."""
        for vx, vy in self._visited_goals:
            if np.sqrt((pos[0] - vx)**2 + (pos[1] - vy)**2) < self._visited_goal_radius:
                return True
        return False

    def _publish_status(self, status):
        msg = ExploreStatus()
        msg.status = status
        self._status_pub.publish(msg)

    # Radius for merging failure tallies of the same drifting frontier goal
    _ABORT_MATCH_RADIUS = 5

    def _count_goal_failure(self, goal):
        """Tally a navigation failure for goal, merging nearby entries.

        Returns the accumulated count. Radius-matched because the re-picked
        goal for the same frontier drifts a few cells between attempts; exact
        keys would never accumulate to the blacklist threshold.
        """
        for entry in self._abort_counts:
            (ex, ey), count = entry
            if np.sqrt((goal[0] - ex)**2 + (goal[1] - ey)**2) < self._ABORT_MATCH_RADIUS:
                entry[1] = count + 1
                return entry[1]
        self._abort_counts.append([goal, 1])
        return 1

    def _clear_goal_failures(self, goal):
        """Drop failure tallies near goal (called on success or blacklist)."""
        self._abort_counts = [
            e for e in self._abort_counts
            if np.sqrt((goal[0] - e[0][0])**2 + (goal[1] - e[0][1])**2)
            >= self._ABORT_MATCH_RADIUS
        ]

    def _register_goal_failure(self, goal, reason):
        """Count a failure; blacklist the goal once it keeps failing."""
        failures = self._count_goal_failure(goal)
        if failures >= self._abort_blacklist_threshold:
            self.get_logger().warn(
                f'Blacklisting goal {goal} after {failures} failures ({reason}).')
            self._visited_goals.append(goal)
            self._clear_goal_failures(goal)
        else:
            self.get_logger().warn(
                f'Goal {goal} failed ({reason}), attempt '
                f'{failures}/{self._abort_blacklist_threshold}.')

    def select_new_frontier_goal(self, robot_pos):
        """Select a new frontier goal."""
        clusters = get_frontier_clusters(self.robot_awareness_map)
        self._publish_frontier_markers(clusters, robot_pos)

        best_frontier = self.find_safe_frontier(robot_pos, clusters)

        if best_frontier is None:
            if self._visited_goals:
                self.get_logger().info('All frontiers blacklisted or blocked — clearing blacklist and retrying.')
                self._visited_goals.clear()
                self._abort_counts.clear()
                best_frontier = self.find_safe_frontier(robot_pos, clusters)
            if best_frontier is None:
                self.get_logger().info('No frontier found - exploration complete!')
                # Signal completion for benchmark tooling; keep the timer
                # alive — new frontiers may still appear as the map grows
                self._publish_status(ExploreStatus.EXPLORATION_COMPLETE)
                return

        # Convert grid coordinates to world coordinates
        best_frontier_world = self.grid_to_world(best_frontier)

        self.get_logger().info(
            f'Selected frontier goal at grid ({best_frontier[0]}, {best_frontier[1]}) '
            f'world ({best_frontier_world[0]:.2f}, {best_frontier_world[1]:.2f})'
        )

        if not self.send_goal_to_nav2(best_frontier_world):
            # Nothing was sent — leaving current_goal unset lets the next
            # tick retry instead of idling until the progress timeout
            # blacklists a goal nav2 never saw
            return
        self.current_goal = best_frontier
        self.goal_start_time = self.get_clock().now()
        self._goal_best_distance = None
    
    def _make_goal_for_cluster(self, cluster, robot_pos):
        """Return the goal position for a cluster.

        Advances PAST the frontier boundary away from the robot so the goal
        lands inside the unexplored room/area rather than at the free/unknown
        edge (which is flush against a wall).

        Unknown cells have OccupancyGrid cost -1, which always passes
        is_goal_safe(), and Nav2 plans through unknown space with
        allow_unknown=true.
        """
        cx = np.mean([p[0] for p in cluster])
        cy = np.mean([p[1] for p in cluster])
        centroid_dist = np.sqrt((cx - robot_pos[0])**2 + (cy - robot_pos[1])**2)

        if centroid_dist > 0:
            # Direction FROM robot TOWARD the frontier (into the unknown region)
            dx = (cx - robot_pos[0]) / centroid_dist
            dy = (cy - robot_pos[1]) / centroid_dist
            advance = self._goal_advance_cells
            rx = int(round(cx + dx * advance))
            ry = int(round(cy + dy * advance))
            h, w = self.robot_awareness_map.shape
            rx = max(0, min(rx, w - 1))
            ry = max(0, min(ry, h - 1))
            # Accept FREE or UNKNOWN — not OBSTACLE
            if self.robot_awareness_map[ry, rx] != OBSTACLE:
                return (rx, ry), centroid_dist

        # Fallback: centroid-nearest frontier cell
        target = min(cluster, key=lambda p: (p[0] - cx)**2 + (p[1] - cy)**2)
        return target, centroid_dist

    def find_safe_frontier(self, robot_pos, clusters=None):
        """Find the best safe, non-blacklisted frontier.

        Selection strategy (tier-based, closest-first):
          Tier 1 — frontiers within near_frontier_radius cells: sorted by
                    closest distance first (greedy local exploration).
          Tier 2 — everything else: sorted by blended distance+size score.

        This guarantees the robot clears its local neighbourhood before
        committing to distant corridors, regardless of corridor size.
        """
        if clusters is None:
            clusters = get_frontier_clusters(self.robot_awareness_map)
        if not clusters:
            return None

        near, far = [], []
        cap = self._max_effective_frontier_size

        for cluster in clusters:
            if len(cluster) < self.min_frontier_size:
                continue
            target, dist = self._make_goal_for_cluster(cluster, robot_pos)
            entry = (cluster, target, dist, len(cluster))
            if dist <= self._near_frontier_radius:
                near.append(entry)
            else:
                far.append(entry)

        if not near and not far:
            return None

        # Tier 1: nearest first, then biggest (greedy local exploration)
        near.sort(key=lambda x: (x[2], -x[3]))

        # Tier 2: blended distance+size for far frontiers
        def far_score(item):
            _, _, dist, size = item
            d_score = np.exp(-dist / max(self._near_frontier_radius, 1))
            s_score = min(size, cap) / cap
            return self.distance_weight * d_score + self.size_weight * s_score

        far.sort(key=far_score, reverse=True)

        candidates = near + far

        for _, target, _, _ in candidates:
            if self._is_blacklisted(target):
                continue
            if self.is_goal_safe(target):
                return target

        # Fallback: best safe goal ignoring blacklist
        for _, target, _, _ in candidates:
            if self.is_goal_safe(target):
                return target

        return None
    
    def _cancel_current_goal(self):
        """Cancel the active Nav2 goal, if any."""
        if self._current_goal_handle is not None:
            cancel_future = self._current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_callback)
            self._current_goal_handle = None

    def _cancel_done_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().warn(f'Goal cancel request failed: {e}')
            return
        if not response.goals_canceling:
            self.get_logger().warn(
                'Nav2 rejected the cancel request; previous goal may still be active.')

    def _publish_frontier_markers(self, clusters, robot_pos):
        """Publish frontier cluster centroids as spheres for RViz."""
        if not self.visualize or self.current_costmap is None:
            return
        msg = MarkerArray()
        now = self.get_clock().now().to_msg()
        frame = self.current_costmap.header.frame_id
        for i, cluster in enumerate(clusters):
            if len(cluster) < self.min_frontier_size:
                continue
            cx, cy = self.grid_to_world((
                int(np.mean([p[0] for p in cluster])),
                int(np.mean([p[1] for p in cluster])),
            ))
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = now
            m.ns = 'frontiers'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = 0.3
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.4
            m.color = ColorRGBA(r=0.0, g=1.0, b=0.5, a=0.9)
            m.lifetime.sec = 2
            msg.markers.append(m)
        # Delete stale markers with higher IDs
        del_m = Marker()
        del_m.action = Marker.DELETEALL
        del_m.ns = 'frontiers'
        if not msg.markers:
            msg.markers.append(del_m)
        self._frontier_pub.publish(msg)

    def check_goal_progress(self):
        """Cancel the goal if the robot has made no progress toward it.

        Stall-based: goal_start_time is reset every time nav2 feedback shows
        distance_remaining improving, so a long traverse that keeps moving is
        never cancelled — only a robot that is genuinely stuck.
        """
        elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9

        if elapsed > self.progress_timeout:
            self.get_logger().warn(
                f'No progress toward goal for {elapsed:.1f}s '
                f'(> {self.progress_timeout}s). Cancelling and selecting new frontier.'
            )
            self._cancel_current_goal()
            if self.current_goal is not None:
                # A stall is a failure, not a visit — blacklist only after
                # repeated failures so one bad approach angle or transient
                # congestion doesn't wipe the frontier's neighbourhood
                self._register_goal_failure(self.current_goal, 'progress stall')
            self.current_goal = None

    def _feedback_callback(self, feedback_msg):
        """Reset the stall timer whenever distance to goal improves."""
        distance = feedback_msg.feedback.distance_remaining
        if distance <= 0.0:
            return
        if self._goal_best_distance is None or \
                distance < self._goal_best_distance - 0.1:
            self._goal_best_distance = distance
            self.goal_start_time = self.get_clock().now()
    
    def grid_to_world(self, grid_pos):
        """Convert grid coordinates to world coordinates."""
        grid_x, grid_y = grid_pos
        origin_x = self.current_costmap.info.origin.position.x
        origin_y = self.current_costmap.info.origin.position.y
        resolution = self.current_costmap.info.resolution
        
        world_x = origin_x + (grid_x + 0.5) * resolution
        world_y = origin_y + (grid_y + 0.5) * resolution
        
        return (world_x, world_y)
    
    def send_goal_to_nav2(self, target_position):
        """Send goal to Nav2 NavigateToPose action. Returns True if sent."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.current_costmap.header.frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = target_position[0]
        goal_msg.pose.pose.position.y = target_position[1]
        goal_msg.pose.pose.position.z = 0.0

        # Set orientation to face the goal
        goal_msg.pose.pose.orientation.w = 1.0

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('NavigateToPose action server not available')
            return False

        send_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback)
        send_future.add_done_callback(self._goal_response_callback)
        return True

    def _goal_response_callback(self, future):
        """Handle goal acceptance; attach result callback."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2, will retry.')
            self.current_goal = None
            return
        self._current_goal_handle = goal_handle
        # Pass the handle through the closure so result callback can ignore stale results
        goal_handle.get_result_async().add_done_callback(
            lambda f, gh=goal_handle: self._goal_result_callback(f, gh)
        )

    def _goal_result_callback(self, future, goal_handle):
        """Handle goal completion — immediately pick a new frontier."""
        # Ignore results from superseded goals (e.g. cancelled due to timeout preemption)
        if goal_handle is not self._current_goal_handle:
            return
        from action_msgs.msg import GoalStatus
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded, selecting next frontier.')
            if self.current_goal is not None:
                self._visited_goals.append(self.current_goal)
                self._clear_goal_failures(self.current_goal)
        else:
            self.get_logger().warn(f'Goal aborted/cancelled (status={status}), selecting new frontier.')
            self._retry_after_wall_time = _time.time() + self._abort_retry_delay
            # Blacklist goals that keep failing so we don't oscillate on unreachable frontiers
            if self.current_goal is not None:
                self._register_goal_failure(self.current_goal, f'status={status}')
        self.current_goal = None
        self._current_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorerNode()
    
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
