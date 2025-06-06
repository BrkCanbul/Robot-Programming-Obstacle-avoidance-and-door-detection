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
                
    
        
        self.x_goal = self.get_parameter("x_goal").value
        self.y_goal = self.get_parameter("y_goal").value
            
        self.k_att = self.get_parameter("K_att").value
        self.k_rep = self.get_parameter("K_rep").value  
        self.rep_field = self.get_parameter("rep_field").value
        self.v_linear_max_ms = self.get_parameter("v_linear_max").value
        self.v_angular_max_rads = self.get_parameter("v_angular_max").value
        
        self.publisher = self.create_publisher(TwistStamped, '/layka_controller/cmd_vel', 10)
        
        self.timer_period = 0.1 ## 10 hertz for control loop
        self.create_timer(self.timer_period,self.control_loop)
        
        self.counter = 0
        self.lastest_scan:LaserScan = None
        self.current_twist = TwistStamped()
        self.last_odometry:Odometry = None
        
        self.get_logger().info(f'Object Avoidance Node Started to goal (x,y) :({self.x_goal},{self.y_goal}) ')



    def lidar_callback(self, msg):
        self.lastest_scan = msg
        
    def odometry_callback(self,msg):
        self.last_odometry = msg
        
        
    def control_loop(self):

        if self.lastest_scan == None:
            return
        
        scan = self.lastest_scan
        F_att = self._compute_att_force()
        F_rep = self._compute_rep_force(scan)
        F_total = F_att + F_rep
        
        twist_command = self.conv_force_to_twist(F_total)
        self.publisher.publish(twist_command)
        self.current_twist = twist_command
    
    def _compute_att_force(self)-> np.ndarray:  
        goal_vector =  np.array([self.x_goal,self.y_goal])
        self.get_logger().info(f"goal vector : {goal_vector}")
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
    
            
        
    
    def conv_force_to_twist(self,F:np.ndarray):
            
        """
        Toplam kuvvet (Fx, Fy) üzerinden açısal ve lineer hız hesaplar.
        v_cmd = k_lin * ||F|| * max(0, cos(angle_error)) formülü,
        sapma 0° iken %100 hız, sapma 90° iken 0 hız, sapma 180° iken de 0 hız üretir.
        """
        Fx, Fy = F[0], F[1]

        
        target_angle = np.arctan2(Fy, Fx)  

        # 2. Açısal hız (P kontroller):
        k_ang = 1.0
        w_cmd = k_ang * target_angle

        # 3. Kuvvetin büyüklüğü:
        mag_F = np.linalg.norm(F)
        if mag_F < 1e-6:
            # Eğer neredeyse sıfır kuvvetse, hareketsiz Twist döndür
            return TwistStamped()

        # 4. Yumuşak lineer hız: sapmadan etkilenir
        #    cos(angle_error) negatif olursa v hemen 0 olacak
        cos_term = np.cos(target_angle)
        forward_factor = max(0.0, cos_term)  # eğer açısal sapma ∈ (90°, 270°) ise 0
        k_lin = 0.2
        v_cmd = k_lin * mag_F * forward_factor

        # 5. Maksimum limit ve tip güvenliği
        v_cmd = np.clip(v_cmd, -self.v_linear_max_ms, self.v_linear_max_ms)
        w_cmd = np.clip(w_cmd, -self.v_angular_max_rads, self.v_angular_max_rads)

        twist = TwistStamped()
        twist.twist.linear.x = float(v_cmd)
        twist.twist.angular.z = float(w_cmd)
        twist.header.frame_id = "base_link"
        twist.header.stamp = self.get_clock().now().to_msg()
        self.current_twist =twist
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