#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped,Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from .pid_Controller import PIDController
from .visualiser import Visualizer


import numpy as np
import math as m
import random as rand

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
            self.odom_callback,
            10)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/layka_controller/cmd_vel', 10)
        
        self.visualiser = Visualizer()
        
        #-- ROBOT
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_heading = 0.0
        
        self.last_scan:LaserScan = None
        
        
        
        #-- PARAMETERS    
        self.declare_parameter("goal_x",10.0)
        self.declare_parameter("goal_y",0.0)
        self.declare_parameter("k_att",1.3)
        self.declare_parameter("k_rep",0.4)
        self.declare_parameter("d0",3.50)
        self.declare_parameter("v_linear_max",0.2)
        self.declare_parameter("w_angular_max",0.8)
        self.declare_parameter("min_distance",0.01)
        self.declare_parameter("max_rep",5.0)
        
        #-- PID PARAMETERS
        self.declare_parameter("Kp_ang",0.30)
        self.declare_parameter("Ki_ang",0.02)
        self.declare_parameter("Kd_ang",0.10)
        self.declare_parameter("Kp_lin",0.60)
        self.declare_parameter("Ki_lin",0.01)
        self.declare_parameter("Kd_lin",0.05)
        self.declare_parameter("deadband_ang",0.05)
        self.declare_parameter("d_thresh",0.20)
        self.declare_parameter("alpha",0.7)
        
        kp_ang = self.get_parameter("Kp_ang").value
        ki_ang = self.get_parameter("Ki_ang").value
        kd_ang = self.get_parameter("Kd_ang").value
        kp_lin = self.get_parameter("Kp_lin").value
        ki_lin = self.get_parameter("Ki_lin").value
        kd_lin = self.get_parameter("Kd_lin").value
        deadband_ang = self.get_parameter("deadband_ang").value
        d_thresh = self.get_parameter("d_thresh").value
        alpha = self.get_parameter("alpha").value        
        
        
        self.pid_controller = PIDController(
            Kp_ang=kp_ang, Ki_ang=ki_ang, Kd_ang=kd_ang,
            Kp_lin=kp_lin, Ki_lin=ki_lin, Kd_lin=kd_lin,
            deadband_ang=deadband_ang, d_thresh=d_thresh, alpha=alpha
        )
        
        
        #- ESCAPE_PARAMS
        self.declare_parameter("max_stuck_count",15)
        self.declare_parameter("stuck_lin_thr",0.2)
        self.declare_parameter("stuck_ang_thr",0.4)
        
        
        
        self.goal_x         = self.get_parameter("goal_x").value
        self.goal_y         = self.get_parameter("goal_y").value
        self.k_att          = self.get_parameter("k_att").value
        self.k_rep          = self.get_parameter("k_rep").value
        self.d0             = self.get_parameter("d0").value
        self.v_linear_max   = self.get_parameter("v_linear_max").value
        self.w_angular_max  = self.get_parameter("w_angular_max").value
        self.min_distance   = self.get_parameter("min_distance").value
        self.max_rep        = self.get_parameter("max_rep").value
        
        
        
        self.max_stuck_count= self.get_parameter("max_stuck_count").value
        self.stuck_lin_thr  = self.get_parameter("stuck_lin_thr").value
        self.stuck_ang_thr  = self.get_parameter("stuck_ang_thr").value
        self.stuck_counter = 0
        
        
        self.control_time   = 0.1
        
        self.create_timer(self.control_time,self.control_loop)
        
        
        self.get_logger().info('Object Avoidance Node Started')



    def calculate_repulsive_force(self,scan:LaserScan):
        x_pts,y_pts = self.scan_to_points(scan)
        
        f_rep_total = np.array([0.0,0.0])
        
        for x,y in zip(x_pts,y_pts):
            d = m.hypot(x,y)
            if d<1e-3:
                continue
            
            d_clamped = max(d,self.min_distance)
            if d_clamped >self.d0:
                continue
            
            mag = self.k_rep * (1.0/d_clamped - 1.0/self.d0) / d_clamped**2
            
            mag = min(mag,self.max_rep)
            f_rep_total += mag * np.array([-x/d_clamped,-y/d_clamped])
            
        return f_rep_total
    
    def calculate_att_force(self):
        x_goal = self.goal_x
        y_goal = self.goal_y
        return self.k_att * np.array([x_goal - self.robot_x,y_goal - self.robot_y])
    
        
    def scan_to_points(self,scan:LaserScan):
        ranges = np.array(scan.ranges)
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        x_pts = []
        y_pts = []
        
        
        for i,d in enumerate(ranges):
            
            if np.isnan(d) or np.isinf(d) or d > self.d0:
                continue
            
            angle = angle_min + i*angle_increment
            x = d * m.cos(angle)
            y = d * m.sin(angle)
            
            x_pts.append(x)
            y_pts.append(y)
            
        return x_pts,y_pts        
            
            
            
        
               
        
    def lidar_callback(self, msg:LaserScan):
        self.last_scan = msg

    def odom_callback(self, msg:Odometry):
        self.robot_x =  msg.pose.pose.position.x
        self.robot_y =  msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_heading = m.atan2(siny_cosp,cosy_cosp)
        
    
    
    def control_loop(self):
        if self.last_scan == None:
            return
        att_force = self.calculate_att_force() ## Wrold frame
        
        rep_force_b = self.calculate_repulsive_force(self.last_scan) ## body frame 
        
        c,s =np.cos(self.robot_heading),np.sin(self.robot_heading)
        R_body_to_word = np.array([[c,-s],[s, c]])
        
        rep_force =R_body_to_word.dot(rep_force_b)
        
        
        
        mag_rep = np.linalg.norm(rep_force)
        if mag_rep >self.max_rep:
            rep_force = (rep_force/mag_rep)*self.max_rep
            
            
        F_net = att_force+rep_force # f_net is in world frame
        
        
        twist = self.pid_controller.compute_twist(
            F_net,robot_yaw=self.robot_heading,
            dt=self.control_time
            ,v_max=self.v_linear_max,
            w_max=self.w_angular_max
        
        )
        #twist = self.compute_twist(F_net)
            
        
        
        
        self.cmd_vel_pub.publish(twist)
        self.visualiser.update(*self.scan_to_points(self.last_scan),F_total=F_net, F_att=att_force,F_rep=rep_force_b,
                               robot_x=self.robot_x,robot_y=self.robot_y)
    
    def compute_twist(self,f_total):
        cos = np.cos(self.robot_heading)
        sin = np.sin(self.robot_heading)
        R_world_to_body = np.array([
            [cos,sin],
            [-sin,cos]
        ])
        
        F_body = R_world_to_body.dot(f_total)
        
        k_lin = 0.5
        k_ang = 0.2
        
        fx_b,fy_b =F_body
        angle_err = np.arctan2(fy_b,fx_b)
        force_mag = m.hypot(fx_b,fy_b)
        v_cmd = k_lin * force_mag* np.cos(angle_err)
        w_cmd = -k_ang*angle_err
        
        v_cmd = float(np.clip(v_cmd, -self.v_linear_max, self.v_linear_max))
        w_cmd = float(np.clip(w_cmd, -self.w_angular_max, self.w_angular_max))
        st = TwistStamped()
        hdr = Header()
        twist = Twist()
        hdr.frame_id ="base_link"
        hdr.stamp = self.get_clock().now().to_msg()
        
        twist.angular.z = w_cmd
        twist.linear.x = v_cmd
        st.header = hdr
        st.twist = twist
        
        return st

def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoidanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.pid_controller.save_hist()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()