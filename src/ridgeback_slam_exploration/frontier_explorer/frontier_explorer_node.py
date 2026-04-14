#!/usr/bin/env python3

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

from navigator import get_best_frontier, update_robot_awareness_from_costmap
from path_finding import a_star
from params import UNKNOWN, OBSTACLE, FREE


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
        
        # Get parameters
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.planner_frequency = self.get_parameter('planner_frequency').value
        self.progress_timeout = self.get_parameter('progress_timeout').value
        self.distance_weight = self.get_parameter('distance_weight').value
        self.size_weight = self.get_parameter('size_weight').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.visualize = self.get_parameter('visualize').value
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create callback groups for concurrent operations
        callback_group_1 = MutuallyExclusiveCallbackGroup()
        callback_group_2 = MutuallyExclusiveCallbackGroup()
        
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
        self.robot_position = None
        self.current_goal = None
        self.goal_start_time = None
        
        self.get_logger().info(f'Frontier Explorer Node initialized')
        self.get_logger().info(f'  Robot Base Frame: {self.robot_base_frame}')
        self.get_logger().info(f'  Costmap Topic: {self.costmap_topic}')
        self.get_logger().info(f'  Planner Frequency: {self.planner_frequency} Hz')
        
    def costmap_callback(self, msg: OccupancyGrid):
        """Receive and process occupancy grid (costmap)."""
        self.current_costmap = msg
        
        # Initialize or reinitialize robot awareness map if size changed
        map_height = msg.info.height
        map_width = msg.info.width
        if self.robot_awareness_map is None or self.robot_awareness_map.shape != (map_height, map_width):
            self.robot_awareness_map = np.full((map_height, map_width), UNKNOWN, dtype=np.int8)
            self.get_logger().info(f'Initialized robot awareness map: {map_width}x{map_height}')
        
        # Convert costmap to robot awareness map
        try:
            self.update_awareness_map(msg)
        except Exception as e:
            self.get_logger().warn(f'Error updating awareness map: {e}')
    
    def update_awareness_map(self, costmap_msg: OccupancyGrid):
        """Convert occupancy grid to robot awareness map."""
        height = costmap_msg.info.height
        width = costmap_msg.info.width
        data = np.array(costmap_msg.data, dtype=np.int8).reshape((height, width))
        
        # Convert occupancy values to our map representation
        # Occupancy grid: -1=unknown, 0=free, 100=obstacle
        for i in range(height):
            for j in range(width):
                value = data[i, j]
                if value == -1:
                    self.robot_awareness_map[i, j] = UNKNOWN
                elif value == 0:
                    self.robot_awareness_map[i, j] = FREE
                else:  # value > 0
                    self.robot_awareness_map[i, j] = OBSTACLE
    
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
    
    def select_new_frontier_goal(self, robot_pos):
        """Select a new frontier goal."""
        best_frontier = get_best_frontier(
            robot_pos,
            self.robot_awareness_map,
            distance_weight=self.distance_weight,
            size_weight=self.size_weight,
            min_frontier_size=self.min_frontier_size
        )
        
        if best_frontier is None:
            self.get_logger().info('No frontier found - exploration complete!')
            return
        
        # Convert grid coordinates to world coordinates
        best_frontier_world = self.grid_to_world(best_frontier)
        
        self.get_logger().info(
            f'Selected frontier goal at grid ({best_frontier[0]}, {best_frontier[1]}) '
            f'world ({best_frontier_world[0]:.2f}, {best_frontier_world[1]:.2f})'
        )
        
        self.send_goal_to_nav2(best_frontier_world)
        self.current_goal = best_frontier
        self.goal_start_time = self.get_clock().now()
    
    def check_goal_progress(self):
        """Check if goal is still valid or if we're stuck."""
        elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
        
        if elapsed > self.progress_timeout:
            self.get_logger().warn(
                f'Goal progress timeout ({elapsed:.1f}s > {self.progress_timeout}s). '
                'Selecting new frontier.'
            )
            self.current_goal = None
    
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
        """Send goal to Nav2 NavigateToPose action."""
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
            return
        
        self.nav_client.send_goal_async(goal_msg)


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
