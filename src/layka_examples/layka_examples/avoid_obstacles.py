import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped,Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

import numpy as np
import math as m




from .visualiser import Visualizer


class ObjectAvoidanceNode(Node):
    def __init__(self):
        super().__init__('object_avoidance_node')
        self.scan_sub = self.create_subscription(
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
        
        self.declare_parameter("goal_x", 5.0)
        self.declare_parameter("goal_y", 0.0)
        
        self.declare_parameter("K_att", 0.4)          # Attraction force constant
        self.declare_parameter("K_rep", 0.1)          # Repulsive force constant
        self.declare_parameter("rep_field", 1.75)      # Repulsive field range
        self.declare_parameter("v_max",0.5)
        self.declare_parameter("w_max",1.0)
        
        
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        
        self.goal_x = self.get_parameter("goal_x").value
        self.goal_y = self.get_parameter("goal_y").value
        
        self.k_att = self.get_parameter("K_att").value
        self.k_rep = self.get_parameter("K_rep").value
        self.rep_field = self.get_parameter("rep_field").value
        self.v_max = self.get_parameter("v_max").value
        self.w_max = self.get_parameter("w_max").value
        
        
        self.control_time = 0.01
        self.last_goal_selection = self.get_clock().now()
        
        
        self.counter = 0
        self.publisher = self.create_publisher(TwistStamped, '/layka_controller/cmd_vel', 10)
        self.create_timer(
            self.control_time,
            self.control_loop
        )
        self.get_logger().info('Object Avoidance Node Started')
        
        self.last_scan = None
        self.visualiser_world = Visualizer()
        
        
        
    def lidar_callback(self, msg):
        
        self.last_scan = msg
        
    def odometry_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        orientation  = msg.pose.pose.orientation
        
        self.robot_theta = self.get_yaw_from_quaternion(orientation)

    def get_yaw_from_quaternion(self, orientation):
        """
        Convert quaternion orientation to yaw angle.
        """
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        
        # Calculate the yaw angle from the quaternion
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = m.atan2(siny_cosp, cosy_cosp)
        
        return yaw        

    def control_loop(self):
        if self.last_scan is None:
            return
        scan:LaserScan = self.last_scan
        
        f_att = self.compute_attraction_force()
        f_rep = self.compute_repulsive_force(scan)
        f_total = f_att + f_rep
        twist_command = self.compute_twist(f_total)
        
        min_distance = min(scan.ranges)
        
        if  min_distance < 0.5:
            
            self.get_logger().warn("Obstacle too close! Stopping robot.")
            
            min_index = scan.ranges.index(min_distance)
            angle_min = scan.angle_min + min_index * scan.angle_increment
            angle = angle_min + m.pi  # Turn away from the obstacle
            #if robot yaw is bakcwards to the obstacle, start liniear movement
            if abs(angle) < m.pi/2:
                twist_command.twist.linear.x = self.v_max
                twist_command.twist.angular.z = 0.0 
            else:
                twist_command.twist.linear.x = 0.0
                twist_command.twist.angular.z = self.w_max * np.sign(self.robot_theta - angle)
            
                
            
            
        
        
        
        
        
        self.publisher.publish(twist_command)
        x_pts,y_pts = self.scan_to_points(self.last_scan)
        
        if self.is_reached_goal():
            self.get_logger().info("Goal Reached!")
            self.select_random_goal()
            self.last_goal_selection = self.get_clock().now()
        else:
            if (self.get_clock().now() - self.last_goal_selection).nanoseconds > 50e9:  # 10 seconds
                self.select_random_goal()
                self.last_goal_selection = self.get_clock().now()
        
        
        self.visualiser_world.update(
            x_points= x_pts,
            y_points= y_pts,
            F_att=self.world_to_body(f_att),
            F_rep=self.body_to_world(f_rep),
            F_total=self.world_to_body(f_total),
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            
        )
        
        
    def is_reached_goal(self):
        """
        Check if the robot has reached the goal position.
        """
        distance = m.sqrt((self.goal_x - self.robot_x)**2 + (self.goal_y - self.robot_y)**2)
        return distance < 0.5
    
    def select_random_goal(self):
        
        """
        Select a random goal position within a specified range.
        """
        self.goal_x = np.random.uniform(-2.0, 2.0)
        self.goal_y = np.random.uniform(-2.0, 2.0)
        self.get_logger().info(f"New Goal: ({self.goal_x}, {self.goal_y})")
        
        
        
        
    def world_to_body(self, force):
        """
        Convert world coordinates to body coordinates.
        """
        sin = m.sin(self.robot_theta)
        cos = m.cos(self.robot_theta)
        R = np.array([[cos, -sin],
                       [sin, cos]])
        return R @ force
    
    def body_to_world(self, force):
        """
        Convert body coordinates to world coordinates.
        """
        sin = m.sin(self.robot_theta)
        cos = m.cos(self.robot_theta)
    
        R = np.array([[cos, sin],
                       [-sin, cos]])
        return R @ force
    
    
    
    
        
    def compute_attraction_force(self):
        """
        Compute the attraction force towards the goal.
        """
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        distance = m.sqrt(dx**2 + dy**2)
        
        if distance < 0.01:
            return np.zeros(2)
        
        force_magnitude = self.get_parameter("K_att").value * distance
        angle = m.atan2(dy, dx)
        
        return np.array([force_magnitude * m.cos(angle), force_magnitude * m.sin(angle)])
    
    def compute_repulsive_force(self,scan:LaserScan):
        """
        Compute the repulsive force based on Lidar scan data.
        """
        f_rep = np.zeros(2)
        x_points, y_points = self.scan_to_points(scan)
        
        for x, y in zip(x_points, y_points):
            d = m.hypot(x, y)
            
            if d> self.rep_field:
                continue
            ang = m.atan2(y, x)
            mag = self.k_rep *(1.0/d - 1.0/self.rep_field)
            f_rep += mag * np.array([m.cos(ang), m.sin(ang)])
        
        f_rep_magnitude = m.sqrt(f_rep[0]**2 + f_rep[1]**2)
        if f_rep_magnitude < 0.01:
            return np.zeros(2)
        
        
        return f_rep
        
    def compute_twist(self, F_total):
        """
        Compute the velocity command based on the total force.
        """
        F_att_x, F_att_y = self.compute_attraction_force()
        F_rep_x, F_rep_y = F_total
        
        # Total force components
        total_force_x = F_att_x + F_rep_x
        total_force_y = F_att_y + F_rep_y
        
        # Calculate the magnitude and angle of the total force
        force_magnitude = m.sqrt(total_force_x**2 + total_force_y**2)
        
        if force_magnitude < 0.01:
            return TwistStamped(header=Header(stamp=self.get_clock().now().to_msg()), twist=Twist())
        
        angle = m.atan2(total_force_y, total_force_x)
        
        # Compute linear and angular velocities
        linear_velocity = min(self.v_max, force_magnitude)
        angular_velocity = min(self.w_max, angle - self.robot_theta)
        
        twist_command = TwistStamped()
        twist = Twist()
        header = Header()
        header.frame_id = "base_link"
        header.stamp = self.get_clock().now().to_msg()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        twist_command.header = header
        twist_command.twist = twist
        
        return twist_command
        
                
        
    def scan_to_points(self,scan:LaserScan)-> tuple[list,list]:
        # Convert scan ranges to x, y points in robot frame, only for FOV -90 to 90 degrees
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        ranges = scan.ranges
        x_points = []
        y_points = []
        for i,d in enumerate(ranges):
            if np.isinf(d) or np.isnan(d) or d <= 0.0:
                continue
            angle = angle_min + i * angle_increment
            if angle < -m.pi/2 or angle > m.pi/2:
                continue
            x = d* m.cos(angle)
            y = d*m.sin(angle)
            x_points.append(x)
            y_points.append(y)
        return x_points, y_points
            
                
            
        
    
    
    
    
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