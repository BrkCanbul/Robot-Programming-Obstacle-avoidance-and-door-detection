#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header

class ObjectAvoidanceNode(Node):
    def __init__(self):
        super().__init__('object_avoidance_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        
        
        self.declare_parameter("K_att",1.0)         ## attractive coef 
        self.declare_parameter("K_rep",1.0)         ## repulsive  coef
        self.declare_parameter("rep_field",1.0)     ## repulsive force affection field
        self.declare_parameter("v_linear_max",0.5)  ## m/s
        self.declare_parameter("v_angular_max",1.0) ## rad/s
        
        self.declare_parameter("x_goal",5.0)
        self.declare_parameter("y_goal",6.0)
        
        self.x_goal = self.get_parameter("x_goal")
        self.y_goal = self.get_parameter("y_goal")
        
        
        self.k_att = self.get_parameter("K_att")
        self.k_rep = self.get_parameter("K_rep")
        self.rep_field = self.get_parameter("rep_field")
        self.v_linear_max_ms = self.get_parameter("v_linear_max")
        self.v_angular_max_rads = self.get_parameter("v_angular_max")
        
        self.publisher = self.create_publisher(TwistStamped, '/layka_controller/cmd_vel', 10)
        
        self.timer_period = 0.1 ## 10 hertz for control loop
        self.counter = 0
        self.lastest_scan:LaserScan = None
        self.current_twist = TwistStamped()
        
        self.get_logger().info('Object Avoidance Node Started')



    def lidar_callback(self, msg):
        
        self.lastest_scan = msg

    def control_loop(self):

        if self.lastest_scan == None:
            return
        
        scan = self.lastest_scan
        
        # TODO: Implement avoidance algorithm uses APF (Artificial potential fileds )
    
    def _compute_att_force(self)-> np.ndarray:  
        goal_vector =  np.array([self.x_goal,self.y_goal])
        return self.k_att * goal_vector
        
        
    
    def _compute_rep_force(self,scan:LaserScan)-> np.ndarray:
        ranges = np.array(scan.ranges)
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        
        f_rep_total = np.zeros(2,dtype=np.float32)
        
        for index,d_i in enumerate(ranges):
            
            if np.isinf(d_i) or np.isnan(d_i):
                continue
            
            if d_i > self.rep_field or d_i <=0.0:
                continue
            
            theta_i = angle_min * index *angle_inc
            
            coeff = (1.0/d_i) - (1.0/self.rep_field)
            
            f_i = self.k_rep * coeff * (1.0/(d_i*d_i))
            
            ux = np.cos(theta_i)
            uy = np.sin(theta_i)
            
            
            f_rep_total +=  f_i *np.array([ux,uy])
            
        return f_rep_total
    
            
        
    
    def conv_force_to_twist(self,Force:np.ndarray):
        eps = 1e-6
        
        target_angle = np.arctan2(Force[0],Force[1])
        
        angle_error = target_angle
        
        
        k_angle = 1.0
        w_cmd = k_angle *angle_error
        
        magnitude_F =np.linalg.norm(Force)
        k_lin = 0.2
        v_cmd = k_lin*magnitude_F
        
        
        max_ang = abs(target_angle)
        
        if max_ang > np.pi/4:
            v_cmd =0.0
            
        v_cmd = np.clip(v_cmd,-self.v_linear_max_ms,self.v_linear_max_ms)
        w_cmd = np.clip(w_cmd,-self.v_angular_max_rads,self.v_angular_max_rads)
        twiststamp = TwistStamped()
        
        twiststamp.twist.linear.x  = float(v_cmd)
        twiststamp.twist.angular.z = float(w_cmd)
        
        
        return twiststamp
            



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