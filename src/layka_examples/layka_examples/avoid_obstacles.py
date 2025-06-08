#!/usr/bin/env python3
import numpy as np
import math
import rclpy
import matplotlib.pyplot as plt
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from .visualiser import Visualizer
from .pid_Controller import PIDController



# class ObjectAvoidanceNode(Node):
#     def __init__(self):
#         super().__init__('object_avoidance_node')
#         self.lidar_sub = self.create_subscription(
#             LaserScan,
#             '/scan',
#             self.lidar_callback,
#             10)
#         self.odom_sub = self.create_subscription(
#             Odometry,
#             '/layka_controller/odom',
#             self.odometry_callback,
#             10
#         )
    
        
#         self.declare_parameter("K_att",1.0)         ## attractive coef 
#         self.declare_parameter("K_rep",1.0)         ## repulsive  coef
#         self.declare_parameter("rep_field",1.0)     ## repulsive force affection field
#         self.declare_parameter("v_linear_max",0.5)  ## m/s
#         self.declare_parameter("v_angular_max",1.0) ## rad/s
        
#         self.declare_parameter("x_goal",5.0)
#         self.declare_parameter("y_goal",6.0) 
                
    
        
#         self.x_goal = self.get_parameter("x_goal").value
#         self.y_goal = self.get_parameter("y_goal").value
        
#         self.robot_gx = 0.0
#         self.robot_gy = 0.0
#         self.robot_yaw = 0.0
            
#         self.k_att = self.get_parameter("K_att").value
#         self.k_rep = self.get_parameter("K_rep").value  
#         self.rep_field = self.get_parameter("rep_field").value
#         self.v_linear_max_ms = self.get_parameter("v_linear_max").value
#         self.v_angular_max_rads = self.get_parameter("v_angular_max").value
#         self.max_force = 5.0
        
#         self.publisher = self.create_publisher(TwistStamped, '/layka_controller/cmd_vel', 10)
        
#         self.timer_period = 0.1 ## 10 hertz for control loop
#         self.create_timer(self.timer_period,self.control_loop)
        
#         self.counter = 0
#         self.lastest_scan:LaserScan = None
#         self.visualizer = Visualizer()
#         self.current_twist = TwistStamped()
    
    
#         self.pid_controller = PIDController()
        
        
#         self.get_logger().info(f'Object Avoidance Node Started to goal (x,y) :({self.x_goal},{self.y_goal}) ')



#     def get_lidar_points(self,scan:LaserScan):
#         angle_min = scan.angle_min
#         angle_inc = scan.angle_increment
#         ranges = np.array(scan.ranges)
        
#         x_points,y_points = [],[]
        
#         for i,distance in enumerate(ranges):
            
#             angle = angle_min + angle_inc*i
            
#             x_points.append(distance * np.cos(angle))
#             y_points.append(distance * np.sin(angle))
            
            
#         return x_points,y_points
            
            
        
        
        
        
        
#     def lidar_callback(self, msg):
#         self.lastest_scan = msg
        
#     def odometry_callback(self,msg:Odometry):
#         self.robot_gx = msg.pose.pose.position.x
#         self.robot_gy = msg.pose.pose.position.y
        
#         q = msg.pose.pose.orientation
        
#         siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        
#         self.robot_yaw = np.arctan2(siny_cosp,cosy_cosp)
        
        
#     def control_loop(self):

#         if self.lastest_scan == None:
#             return
        
#         scan = self.lastest_scan
#         F_att = self._compute_att_force()
        
#         if np.linalg.norm(F_att)>self.max_force:
#             F_att = F_att / np.linalg.norm(F_att)*self.max_force
            
            
#         F_rep = self._compute_rep_force(scan)
        
#         if np.linalg.norm(F_rep)>self.max_force:
#             F_rep = F_rep / np.linalg.norm(F_rep)*self.max_force
        
#         F_total = F_att + F_rep
        
#         self.visualizer.update(*self.get_lidar_points(scan),
#                                F_att=F_att,
#                                F_rep=F_rep,
#                                F_total=F_total)
        
#         twist_command = self.conv_force_to_twist(F_total)
#         self.publisher.publish(twist_command)
#         self.current_twist = twist_command
    
#     def _compute_att_force(self)-> np.ndarray:  
        
#         # dx = self.x_goal - self.robot_gx
#         # dy = self.y_goal - self.robot_gy
        
#         # goal_g = np.array([dx,dy])
#         # F_att_global = self.k_att * goal_g
        
#         # psi = self.robot_yaw
        
#         # r_inv = np.array(
#         #     [[math.cos(psi),math.sin(psi)],
#         #     [-math.sin(psi),math.cos(psi)]]
#         # )
#         # F_att_body = r_inv.dot(F_att_global)
        
#         return np.array([5,-5])
        
        
    
#     def _compute_rep_force(self,scan:LaserScan)-> np.ndarray:
#         ranges = np.array(scan.ranges)
#         angle_min = scan.angle_min
#         angle_inc = scan.angle_increment
        
#         f_rep_total = np.zeros(2,dtype=np.float32)
        
#         x_points = []
#         y_points = []
        
        
        
#         for index,d_i in enumerate(ranges):
            
#             if np.isinf(d_i) or np.isnan(d_i):
#                 continue
            
#             if d_i > self.rep_field or d_i <=0.0:
#                 continue
            
#             if d_i <=0.1:
#                 d_i = 0.1
            
#             theta_i = angle_min * index *angle_inc
            
#             coeff = (1.0/d_i) - (1.0/self.rep_field)

#             f_i = self.k_rep * coeff * (1.0/(d_i*d_i))

            
#             ux = np.cos(theta_i)
#             uy = np.sin(theta_i)
            
                        
#             x_points.append(ux)
#             y_points.append(uy)
                        
#             f_rep_total +=  f_i * np.array([ux,uy])
#         return f_rep_total
    
#     def show_obstacles_plot(self,x_points:list,y_points:list):
#        plt.figure(figsize=(6, 6))
#        plt.plot(0, 0, 'ro', label='Robot')
#        plt.scatter(x_points, y_points, c='b', label='Lidar')
#        plt.axis('equal')
#        plt.grid(True)
#        plt.legend()
#        plt.title("Lidar Algıları (Global Koordinatlarda)")
#        plt.show()
        
    
#     def conv_force_to_twist(self,F:np.ndarray):
            
#         """
#         Toplam kuvvet (Fx, Fy) üzerinden açısal ve lineer hız hesaplar.
#         v_cmd = k_lin * ||F|| * max(0, cos(angle_error)) formülü,
#         sapma 0° iken %100 hız, sapma 90° iken 0 hız, sapma 180° iken de 0 hız üretir.
#         """
#         # Fx, Fy = F[0], F[1]
#         Fx,Fy = F[0],F[1]
        
#         target_angle = np.arctan2(Fy,Fx)
#         angle_error = ((target_angle - self.robot_yaw)+np.pi)%(2*np.pi)-np.pi
        
#         force_mag = np.linalg.norm(F)
#         force_mag = min(force_mag,5.0)
        
#         K_angular = 1.5
#         K_linear = 0.5 if abs(angle_error)<np.pi / 6 else 0.0
#         stamped = TwistStamped()
#         from std_msgs.msg import Header
        
#         header = Header()
#         header.frame_id = "base_link"
#         header.stamp = self.get_clock().now().to_msg()
#         stamped.header = header
#         stamped.twist.linear.x = K_linear *force_mag
#         stamped.twist.angular.z = K_angular* angle_error
        
#         stamped.twist.linear.x = float(np.clip(stamped.twist.linear.x,-self.v_linear_max_ms, self.v_linear_max_ms))
#         stamped.twist.angular.z = float(np.clip(stamped.twist.angular.z, -self.v_angular_max_rads, self.v_angular_max_rads)) 
        
#         return stamped
#         return self.pid_controller.compute_twist(
#             F=F,
#             robot_yaw=self.robot_yaw,
#             v_max=self.v_linear_max_ms,
#             w_max=self.v_angular_max_rads
#         )
        
        
class ObjectAvoidanceNode(Node):
    def __init__(self):
        super().__init__('object_avoidance_node')

        # Subscription & Publisher
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Odometry, '/layka_controller/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/layka_controller/cmd_vel', 10)

        # Parametreler
        self.declare_parameter('x_goal', 4.0)
        self.declare_parameter('y_goal', 5.0)
        self.declare_parameter('K_att', 1.0)
        self.declare_parameter('K_rep', 1.0)
        self.declare_parameter('rep_field', 1.5)
        self.declare_parameter('v_linear_max', 0.4)   # m/s
        self.declare_parameter('v_angular_max', 1.0)  # rad/s
        self.declare_parameter('K_lin', 0.5)
        self.declare_parameter('K_ang', 1.5)
        self.declare_parameter('stuck_threshold', 0.05)
        self.declare_parameter('noise_scale', 0.3)

        # Parametre değerlerini al
        self.x_goal = self.get_parameter('x_goal').value
        self.y_goal = self.get_parameter('y_goal').value
        self.K_att = self.get_parameter('K_att').value
        self.K_rep = self.get_parameter('K_rep').value
        self.rep_field = self.get_parameter('rep_field').value
        self.v_linear_max = self.get_parameter('v_linear_max').value
        self.v_angular_max = self.get_parameter('v_angular_max').value
        self.K_lin = self.get_parameter('K_lin').value
        self.K_ang = self.get_parameter('K_ang').value
        self.stuck_threshold = self.get_parameter('stuck_threshold').value
        self.noise_scale = self.get_parameter('noise_scale').value

        # Durum değişkenleri
        self.latest_scan = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.stuck_counter = 0

        # Görselleştirici (isteğe bağlı)
        self.visualizer = Visualizer()

        # Kontrol döngüsü
        timer_period = 0.1  # 10 Hz
        self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(
            f'ObjectAvoidanceNode started: goal=({self.x_goal:.2f},{self.y_goal:.2f})'
        )

    def lidar_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.robot_x = p.x
        self.robot_y = p.y
        # Quaternion -> yaw
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def control_loop(self):
        if self.latest_scan is None:
            return

        # 1) Kuvvetleri hesapla
        F_att = self._compute_att_force()
        F_rep = self._compute_rep_force(self.latest_scan)

        # 2) Kuvvetleri normalize / clamp et
        max_F = max(np.linalg.norm(F_att), np.linalg.norm(F_rep), 1e-6)
        if np.linalg.norm(F_att) > max_F:
            F_att = F_att / np.linalg.norm(F_att) * max_F
        if np.linalg.norm(F_rep) > max_F:
            F_rep = F_rep / np.linalg.norm(F_rep) * max_F

        F_total = F_att + F_rep

        # 3) Lokal minimumdan kaçma: çok küçük net kuvvet varsa gürültü ekle
        if np.linalg.norm(F_total) < self.stuck_threshold:
            self.get_logger().info("STUCK")
            noise = np.random.uniform(-1, 1, size=2) * self.noise_scale
            F_total += noise
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0

        # 4) Görselleştir (opsiyonel)
        pts_x, pts_y = self._extract_points(self.latest_scan)
        self.visualizer.update(pts_x, pts_y,
                               F_att=F_att, F_rep=F_rep, F_total=F_total,
                               robot_x = self.robot_x, robot_y = self.robot_y)

        # 5) Twist komutuna çevir ve yayınla
        cmd = self._force_to_twist(F_total)
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        
        self.cmd_pub.publish(cmd)

    def _compute_att_force(self) -> np.ndarray:
        # Çekici kuvvet: F_att = K_att * (goal - position)
        dx = self.x_goal - self.robot_x
        dy = self.y_goal - self.robot_y
        Fg = np.array([dx, dy]) * self.K_att
        # Dünya -> robot gövde ekseni dönüşümü
        c, s = math.cos(self.robot_yaw), math.sin(self.robot_yaw)
        R_inv = np.array([[ c,  s],
                          [-s,  c]])
        return R_inv.dot(Fg)

    def _compute_rep_force(self, scan: LaserScan) -> np.ndarray:
        ranges = np.array(scan.ranges)
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment

        F_rep_total = np.zeros(2, dtype=float)
        for i, d in enumerate(ranges):
            if not np.isfinite(d) or d <= 0.0 or d > self.rep_field:
                continue
            # Öz: d küçüldükçe itme kuvveti artar
            coeff = (1.0 / d - 1.0 / self.rep_field)
            f_mag = self.K_rep * coeff / d 
            theta = angle_min + i * angle_inc
            ux, uy = math.cos(theta), math.sin(theta)
            F_rep_total += f_mag * np.array([ux, uy])
        return F_rep_total

    def _extract_points(self, scan: LaserScan):
        """ Görselleştirme için Lidar nokta koordinatları """
        ranges = np.array(scan.ranges)
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        xs, ys = [], []
        for i, d in enumerate(ranges):
            if np.isfinite(d) and d > 0.0:
                theta = angle_min + i * angle_inc
                xs.append(d * math.cos(theta))
                ys.append(d * math.sin(theta))
        return xs, ys

    def _force_to_twist(self, F: np.ndarray) -> TwistStamped:
        """ Toplam kuvveti (Fx, Fy) lineer ve açısal hıza dönüştür """
        Fx, Fy = F
        norm = math.hypot(Fx, Fy)
        # Hedef açısı (gövde ekseninde)
        target_angle = math.atan2(Fy, Fx)
        
        # Lineer hız: norm * cos(θ) (θ sapması 90°'de 0)
        v = self.K_lin * norm * max(0.0, math.cos(target_angle))
        w = self.K_ang * target_angle

        # Limitler
        v = float(np.clip(v, -self.v_linear_max, self.v_linear_max))
        w = float(np.clip(w, -self.v_angular_max, self.v_angular_max))

        cmd = TwistStamped()
        cmd.twist.linear.x = v
        cmd.twist.angular.z = w
        return cmd       

           



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