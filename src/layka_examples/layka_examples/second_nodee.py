#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class ObjectAvoidanceNode(Node):
    def __init__(self):
        super().__init__('object_avoidance_node')
        
        # Subscribers for Lidar and Odometry
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/layka_controller/odom',
            self.odometry_callback,
            10
        )
    
        # Declare parameters with tuned values
        self.declare_parameter("K_rep", 0.015)          # Reduced repulsive force for smoother behavior
        self.declare_parameter("rep_field", 0.75)      # Increased range to detect wider openings
        self.declare_parameter("v_linear_max", 0.8)   # Max linear speed (m/s)
        self.declare_parameter("v_angular_max", 1.2)  # Reduced max angular speed for smoother turns
        self.declare_parameter("default_linear_speed", 0.5)  # Slightly increased default speed
        self.declare_parameter("stuck_threshold", 0.05)  # Threshold for stuck detection
        self.declare_parameter("stuck_time", 2.0)      # Reduced time to detect stuck
        self.declare_parameter("min_distance", 0.6)    # Slightly increased min distance to walls
        
        # Retrieve parameter values
        self.k_rep = self.get_parameter("K_rep").value  
        self.rep_field = self.get_parameter("rep_field").value
        self.v_linear_max_ms = self.get_parameter("v_linear_max").value
        self.v_angular_max_rads = self.get_parameter("v_angular_max").value
        self.default_linear_speed = self.get_parameter("default_linear_speed").value
        self.stuck_threshold = self.get_parameter("stuck_threshold").value
        self.stuck_time = self.get_parameter("stuck_time").value
        self.min_distance = self.get_parameter("min_distance").value
        
        # Publisher for velocity commands
        self.publisher = self.create_publisher(TwistStamped, '/layka_controller/cmd_vel', 10)
        
        # Control loop timer (20 Hz for smoother control)
        self.timer_period = 0.05
        self.create_timer(self.timer_period, self.control_loop)
        
        # Initialize variables
        self.lastest_scan = None
        self.current_twist = TwistStamped()
        self.robot_gx = 0.0
        self.robot_gy = 0.0
        self.last_position = None
        self.stuck_timer = 0.0
        
        self.get_logger().info('Object Avoidance Node Started with Force Visualization')

    def lidar_callback(self, msg):
        """Callback to store the latest Lidar scan data."""
        self.lastest_scan = msg
        
    def odometry_callback(self, msg: Odometry):
        """Callback to update robot's position from odometry."""
        self.robot_gx = msg.pose.pose.position.x
        self.robot_gy = msg.pose.pose.position.y

    def control_loop(self):
        """Main control loop to compute, visualize, and publish velocity commands."""
        if self.lastest_scan is None:
            return
        
        scan = self.lastest_scan
        
        # Compute repulsive force and get Lidar points for visualization
        F_rep = self._compute_rep_force(scan)
        x_points, y_points = self.scan_to_points(scan)
        
        # Visualize forces and Lidar points
        self.visualize_forces(x_points, y_points, F_rep)
        
        # Check if the robot is stuck and handle it
        if self.is_stuck():
            self.handle_stuck_situation()
        else:
            self.stuck_timer = 0.0
        
        # Compute and publish velocity commands
        twist_command = self.compute_twist(F_rep)
        self.publisher.publish(twist_command)
        self.current_twist = twist_command
    
    def scan_to_points(self, scan: LaserScan) -> tuple[np.ndarray, np.ndarray]:
        """Convert Lidar scan data to Cartesian points."""
        ranges = np.array(scan.ranges)
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        
        angles = angle_min + np.arange(len(ranges)) * angle_inc
        valid_indices = np.where((np.isfinite(ranges)) & (ranges > 0.0) & (ranges <= self.rep_field))[0]
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]
        x_points = ranges * np.cos(angles)
        y_points = ranges * np.sin(angles)
        
        return x_points, y_points
    
    def visualize_forces(self, x_points, y_points, F_rep):
        """Create a scatter plot of Lidar points and repulsive force vector."""
        # Prepare Lidar points dataset
        lidar_data = [
            {"x": float(x), "y": float(y)} for x, y in zip(x_points, y_points)
        ]
        
        # Prepare repulsive force vector (scaled for visibility)
        scale = 0.5  # Scale factor for visualization
        f_rep_x, f_rep_y = F_rep[0] * scale, F_rep[1] * scale
    
    def is_stuck(self):
        """Detect if the robot is stuck based on movement."""
        if self.last_position is None:
            self.last_position = (self.robot_gx, self.robot_gy)
            return False
        
        current_position = (self.robot_gx, self.robot_gy)
        distance_moved = np.hypot(current_position[0] - self.last_position[0], 
                                  current_position[1] - self.last_position[1])
        speed = distance_moved / self.timer_period
        self.last_position = current_position
        
        if speed < self.stuck_threshold:
            self.stuck_timer += self.timer_period
            if self.stuck_timer > self.stuck_time:
                self.get_logger().info('Robot detected as stuck')
                return True
        else:
            self.stuck_timer = 0.0
        return False
    
    def handle_stuck_situation(self):
        """Handle stuck situation by turning towards the widest open space."""
        scan = self.lastest_scan
        if scan is None:
            return
        
        ranges = np.array(scan.ranges)
        valid_ranges = np.where(np.isfinite(ranges) & (ranges > 0.0), ranges, -1.0)
        
        # Use a larger window to identify wider openings (e.g., doorways)
        window_size = 30  # Increased window size for better detection of wide openings
        smoothed_ranges = np.convolve(valid_ranges, np.ones(window_size)/window_size, mode='valid')
        max_index = np.argmax(smoothed_ranges)
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        target_angle = angle_min + (max_index + window_size // 2) * angle_inc
        
        # Smooth the angular velocity for better turning behavior
        twist = TwistStamped()
        angular_speed = float(np.clip(target_angle * 0.5, -self.v_angular_max_rads, self.v_angular_max_rads))
        twist.twist.angular.z = angular_speed
        twist.twist.linear.x = float(self.default_linear_speed * 0.5)  # Move forward slowly while turning
        twist.header.frame_id = "base_link"
        twist.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(twist)
        self.get_logger().info(f'Robot is stuck, turning towards widest open space at angle {target_angle:.2f} rad')
    
    def _compute_rep_force(self, scan: LaserScan) -> np.ndarray:
        """Compute repulsive force with simplified logic for smoother navigation."""
        ranges = np.array(scan.ranges)
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        
        f_rep_total = np.zeros(2, dtype=np.float32)
        
        for index, d_i in enumerate(ranges):
            theta_i = angle_min + index * angle_inc
            
            if np.isinf(d_i) or np.isnan(d_i) or d_i > self.rep_field or d_i <= 0.0:
                continue
            
            # Simplified force calculation: stronger near min_distance, weaker farther
            coeff = (1.0 / max(d_i, self.min_distance)) - (1.0 / self.rep_field)
            f_i = self.k_rep * coeff
            
            ux = np.cos(theta_i)
            uy = np.sin(theta_i)
            
            f_rep_total += -f_i * np.array([ux, uy])
        
        return f_rep_total
    
    def compute_twist(self, F_rep: np.ndarray):
        """Convert repulsive force to velocity commands, favoring open paths."""
        v_cmd = self.default_linear_speed
        w_cmd = 0.0
        
        if np.linalg.norm(F_rep) > 1e-6:
            angle_rep = np.arctan2(F_rep[1], F_rep[0])
            w_cmd = -angle_rep * 0.6  # Reduced gain for smoother turning
            v_cmd *= max(0.4, 1.0 - np.abs(angle_rep) / (np.pi / 2))  # Slightly increased min speed
        
        v_cmd = np.clip(v_cmd, -self.v_linear_max_ms, self.v_linear_max_ms)
        w_cmd = np.clip(w_cmd, -self.v_angular_max_rads, self.v_angular_max_rads)
        
        twist = TwistStamped()
        twist.twist.linear.x = float(v_cmd)
        twist.twist.angular.z = float(w_cmd)
        twist.header.frame_id = "base_link"
        twist.header.stamp = self.get_clock().now().to_msg()
        return twist

def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoidanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()