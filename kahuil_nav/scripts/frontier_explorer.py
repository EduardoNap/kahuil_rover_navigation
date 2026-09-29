#!/usr/bin/env python3
"""
Frontier Explorer v5 for Kahuil RoboCup Rescue
Ported from rastec project — adapted to kahuil global topics.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import LaserScan

import numpy as np
from scipy import ndimage
import math
import time
import random
import threading
import sys
import select
import tty
import termios


class FrontierExplorer(Node):
    """Explorer with continuous wall monitoring and multi-criteria frontier scoring"""

    UNKNOWN = -1
    FREE_THRESHOLD = 50
    OCCUPIED_THRESHOLD = 65

    def __init__(self):
        super().__init__('frontier_explorer')

        # Tunable parameters
        self.min_frontier_size = 5
        self.min_frontier_distance = 1.0
        self.max_frontier_distance = 8.0
        self.visited_radius = 0.4

        # Wall detection thresholds
        self.wall_danger_distance = 0.7
        self.wall_warning_distance = 1.2

        # State
        self.current_map = None
        self.map_info = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.is_navigating = False
        self.exploration_complete = False
        self.frontiers = []
        self.visited_positions = []
        self.current_goal = None
        self.nav_goal_handle = None

        # Pause control
        self.paused = False
        self.keyboard_thread = None
        self.running = True

        # Laser data
        self.min_front_distance = float('inf')
        self.min_left_distance = float('inf')
        self.min_right_distance = float('inf')
        self.laser_data_received = False

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)

        # Subscribers — kahuil uses global topics (no namespace)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)

        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)

        # Action client
        self.nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')

        # Publishers
        self.frontier_pub = self.create_publisher(
            MarkerArray, '/frontiers', 10)

        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        # Exploration timer
        self.explore_timer = self.create_timer(1.0, self.explore_callback)

        # Continuous wall monitoring (every 200ms)
        self.wall_monitor_timer = self.create_timer(0.2, self.wall_monitor_callback)

        # Keyboard listener thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()

        self.get_logger().info('Frontier Explorer v5 - Kahuil')
        self.get_logger().info(f'  Danger distance: {self.wall_danger_distance}m')
        self.get_logger().info(f'  Warning distance: {self.wall_warning_distance}m')
        self.get_logger().info('')
        self.get_logger().info('  CONTROLS:')
        self.get_logger().info('  [SPACE] - Pause/Resume exploration')
        self.get_logger().info('  [Q]     - Quit')
        self.get_logger().info('')
        self.print_status()

    def laser_callback(self, msg: LaserScan):
        """Process laser and compute sector distances"""
        if len(msg.ranges) < 10:
            return

        self.laser_data_received = True
        n = len(msg.ranges)

        front_start = n // 3
        front_end = 2 * n // 3

        def safe_min(ranges):
            valid = [r for r in ranges if msg.range_min < r < msg.range_max]
            return min(valid) if valid else float('inf')

        self.min_front_distance = safe_min(msg.ranges[front_start:front_end])
        self.min_right_distance = safe_min(msg.ranges[:front_start])
        self.min_left_distance = safe_min(msg.ranges[front_end:])

    def wall_monitor_callback(self):
        """Continuous wall monitoring — soft evasion via Nav2 costmaps"""
        if not self.laser_data_received:
            return

        if self.min_front_distance < self.wall_danger_distance:
            self.get_logger().warn(
                f'Obstacle close: {self.min_front_distance:.2f}m - Nav2 handling evasion',
                throttle_duration_sec=2.0
            )
            return

        if self.min_front_distance < self.wall_warning_distance:
            self.get_logger().info(
                f'Obstacle ahead: {self.min_front_distance:.2f}m',
                throttle_duration_sec=3.0
            )

    def get_robot_position(self):
        """Get robot position from TF"""
        try:
            transform = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            self.robot_x = transform.transform.translation.x
            self.robot_y = transform.transform.translation.y
            return True
        except Exception:
            return False

    def keyboard_listener(self):
        """Listen for keyboard input to pause/resume"""
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while self.running:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)

                    if key == ' ':
                        self.paused = not self.paused
                        self.print_status()
                    elif key.lower() == 'q':
                        self.get_logger().info('Shutting down...')
                        self.running = False
                        rclpy.shutdown()
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def print_status(self):
        """Print current status"""
        status = "PAUSED" if self.paused else "EXPLORING"
        self.get_logger().info('')
        self.get_logger().info(f'  State: {status}')
        if self.paused:
            self.get_logger().info('  -> Use RViz "2D Nav Goal" to move the robot')
            self.get_logger().info('  -> Press SPACE to resume exploration')
        else:
            self.get_logger().info('  -> Press SPACE to pause')
        self.get_logger().info('')

    def explore_callback(self):
        """Exploration cycle"""
        if self.paused:
            return

        if self.current_map is None:
            self.get_logger().info('Waiting for map...', throttle_duration_sec=5.0)
            return

        if self.is_navigating or self.exploration_complete:
            return

        if not self.get_robot_position():
            return

        self.mark_position_visited(self.robot_x, self.robot_y)

        self.frontiers = self.detect_frontiers()
        self.publish_frontier_markers()

        valid_frontiers = self.filter_frontiers()

        if len(valid_frontiers) == 0:
            if len(self.frontiers) == 0:
                self.get_logger().info('Exploration complete!')
                self.exploration_complete = True
            else:
                for f in self.frontiers:
                    x, y = f['centroid']
                    reasons = []
                    if f['distance'] < self.min_frontier_distance:
                        reasons.append(f"too close ({f['distance']:.1f}m)")
                    if f['distance'] > self.max_frontier_distance:
                        reasons.append(f"too far ({f['distance']:.1f}m)")
                    if f['clearance'] < 0.7:
                        reasons.append(f"low clearance ({f['clearance']:.1f}m)")
                    if self.is_position_visited(x, y):
                        reasons.append("already visited")

                    self.get_logger().info(f'   Frontier ({x:.1f}, {y:.1f}): {", ".join(reasons)}')

                self.get_logger().info('Clearing history to continue')
                self.visited_positions = []

                self.execute_exploratory_spin()
            return

        self.get_logger().info(f'{len(valid_frontiers)} valid frontiers')

        best = self.select_best_frontier(valid_frontiers)
        if best:
            self.navigate_to_frontier(best)

    def map_callback(self, msg: OccupancyGrid):
        """Map callback"""
        self.map_info = msg.info
        self.current_map = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

    def is_position_visited(self, x, y):
        """Check if position was visited"""
        for vx, vy in self.visited_positions:
            if math.sqrt((x - vx)**2 + (y - vy)**2) < self.visited_radius:
                return True
        return False

    def mark_position_visited(self, x, y):
        """Mark position as visited"""
        if not self.is_position_visited(x, y):
            self.visited_positions.append((x, y))
            if len(self.visited_positions) > 100:
                self.visited_positions = self.visited_positions[-50:]

    def execute_exploratory_spin(self):
        """Exploratory spin to discover new areas"""
        self.get_logger().info('Executing exploratory spin...')
        twist = Twist()
        twist.angular.z = 0.8

        for _ in range(30):  # 3 seconds
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)

        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Spin complete')

    def detect_frontiers(self):
        """Detect frontiers between free and unknown space"""
        if self.current_map is None:
            return []

        free_mask = (self.current_map >= 0) & (self.current_map < self.FREE_THRESHOLD)
        unknown_mask = self.current_map == self.UNKNOWN
        occupied_mask = self.current_map >= self.OCCUPIED_THRESHOLD

        kernel = np.ones((3, 3), dtype=np.uint8)
        unknown_dilated = ndimage.binary_dilation(unknown_mask, kernel)
        frontier_mask = free_mask & unknown_dilated

        obstacle_distance = ndimage.distance_transform_edt(~occupied_mask)
        min_cells = 0.7 / self.map_info.resolution
        frontier_mask = frontier_mask & (obstacle_distance > min_cells)

        labeled, num_features = ndimage.label(frontier_mask)

        frontiers = []
        for i in range(1, num_features + 1):
            points = np.argwhere(labeled == i)
            if len(points) >= self.min_frontier_size:
                centroid = points.mean(axis=0)
                world_x = self.map_info.origin.position.x + centroid[1] * self.map_info.resolution
                world_y = self.map_info.origin.position.y + centroid[0] * self.map_info.resolution
                dist = math.sqrt((world_x - self.robot_x)**2 + (world_y - self.robot_y)**2)
                clearance = np.mean(obstacle_distance[points[:, 0], points[:, 1]]) * self.map_info.resolution

                frontiers.append({
                    'centroid': (world_x, world_y),
                    'size': len(points),
                    'distance': dist,
                    'clearance': clearance,
                })

        return frontiers

    def filter_frontiers(self):
        """Filter invalid frontiers"""
        return [f for f in self.frontiers
                if self.min_frontier_distance < f['distance'] < self.max_frontier_distance
                and f['clearance'] > 0.7
                and not self.is_position_visited(f['centroid'][0], f['centroid'][1])]

    def select_best_frontier(self, frontiers):
        """Select best frontier — prioritizes unexplored areas"""
        if not frontiers:
            return None

        def score(f):
            dist = f['distance']

            if 1.5 < dist < 4.0:
                dist_score = 1.0
            elif dist < 1.5:
                dist_score = 0.3
            else:
                dist_score = 0.6

            size_score = min(f['size'] / 20.0, 1.5)
            clearance_score = min(f['clearance'] / 1.0, 1.0)
            unknown_ratio = self.calculate_unknown_ratio_near_frontier(f)
            random_factor = random.uniform(0.95, 1.05)

            total_score = (
                dist_score * 0.2 +
                size_score * 0.3 +
                clearance_score * 0.15 +
                unknown_ratio * 0.35
            ) * random_factor

            return total_score

        best = max(frontiers, key=score)
        self.get_logger().info(f'Selected: ({best["centroid"][0]:.2f}, {best["centroid"][1]:.2f}) - size:{best["size"]}')
        return best

    def calculate_unknown_ratio_near_frontier(self, frontier):
        """Calculate ratio of unknown space near the frontier"""
        if self.current_map is None:
            return 0.0

        x, y = frontier['centroid']

        map_x = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        map_y = int((y - self.map_info.origin.position.y) / self.map_info.resolution)

        if not (0 <= map_x < self.current_map.shape[1] and 0 <= map_y < self.current_map.shape[0]):
            return 0.0

        radius_cells = int(2.0 / self.map_info.resolution)

        y_min = max(0, map_y - radius_cells)
        y_max = min(self.current_map.shape[0], map_y + radius_cells)
        x_min = max(0, map_x - radius_cells)
        x_max = min(self.current_map.shape[1], map_x + radius_cells)

        area = self.current_map[y_min:y_max, x_min:x_max]

        if area.size == 0:
            return 0.0

        unknown_count = np.sum(area == self.UNKNOWN)
        total_count = area.size

        return unknown_count / total_count

    def navigate_to_frontier(self, frontier):
        """Navigate to frontier via Nav2"""
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            return

        x, y = frontier['centroid']
        self.mark_position_visited(x, y)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Navigating to: ({x:.2f}, {y:.2f})')
        self.is_navigating = True
        self.current_goal = (x, y)

        future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.nav_feedback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Goal accepted"""
        self.nav_goal_handle = future.result()
        if not self.nav_goal_handle.accepted:
            self.is_navigating = False
            return
        result_future = self.nav_goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_feedback(self, feedback_msg):
        """Navigation feedback"""
        self.get_logger().info(
            f'   Dist: {feedback_msg.feedback.distance_remaining:.2f}m',
            throttle_duration_sec=3.0)

    def nav_result_callback(self, future):
        """Navigation completed"""
        self.is_navigating = False
        self.nav_goal_handle = None
        self.get_logger().info('Navigation completed')

    def publish_frontier_markers(self):
        """Publish visualization markers"""
        marker_array = MarkerArray()

        delete = Marker()
        delete.action = Marker.DELETEALL
        marker_array.markers.append(delete)

        for i, f in enumerate(self.frontiers):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'frontiers'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = f['centroid'][0]
            m.pose.position.y = f['centroid'][1]
            m.pose.position.z = 0.3
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.3

            if self.is_position_visited(f['centroid'][0], f['centroid'][1]):
                m.color.r, m.color.g, m.color.b = 0.5, 0.5, 0.5
            else:
                m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.0
            m.color.a = 0.8
            marker_array.markers.append(m)

        for i, (vx, vy) in enumerate(self.visited_positions[-20:]):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'visited'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = vx
            m.pose.position.y = vy
            m.pose.position.z = 0.05
            m.scale.x = m.scale.y = self.visited_radius * 2
            m.scale.z = 0.1
            m.color.r, m.color.g, m.color.b = 0.0, 0.0, 1.0
            m.color.a = 0.3
            marker_array.markers.append(m)

        self.frontier_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    explorer = FrontierExplorer()
    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass
    finally:
        explorer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
