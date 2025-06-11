#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped

import numpy as np
import math

from .pid_Controller import PIDController
from .visualiser import Visualizer

class ObjectAvoidanceNode(Node):
    def __init__(self):
        super().__init__('object_avoidance_node_2')
        self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        self.publisher = self.create_publisher(
            TwistStamped,
            '/layka_controller/cmd_vel',
            10
        )

        self.visualiser = Visualizer()

        # Parametreler
        self.k_att = 6.0    # çekici kuvvet katsayısı
        self.k_rep = 0.2   # itici kuvvet katsayısı
        self.rep_range = 1.0  # itici kuvvet etki menzili (metre)
        
        self.max_linear_velocity = 0.4  # Maksimum lineer hız (m/s)
        self.max_angular_velocity = 0.8  # Maksimum açısal hız (rad/s)
        

    def lidar_callback(self, msg: LaserScan):
        # 1) Lidar verilerini uygun formata çevir
        ranges = np.array(msg.ranges)
        # Okuma sayısına göre açı dizisi oluştur
        num_readings = len(ranges)
        angles = msg.angle_min + np.arange(num_readings) * msg.angle_increment

        ranges = np.array(msg.ranges)

        # 2) Potansiyel alan kuvvetlerini hesapla
        F_att = self.compute_attractive_force()
        F_rep = self.compute_repulsive_force(angles, ranges)
        F_total = F_att + F_rep

        # 3) Twist mesajına dönüştür
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        
        linear_vel, angular_vel = self.compute_twist(F_total)

        w_command = np.clip(angular_vel, -self.max_angular_velocity, self.max_angular_velocity)
        v_command = np.clip(linear_vel, -self.max_linear_velocity, self.max_linear_velocity)
        

        twist.twist.linear.x = 0.4
        twist.twist.angular.z = -float(w_command)  # Negatif çünkü ROS'ta saat yönünün tersine pozitif açısal hız

        self.publisher.publish(twist)
        
        # Görselleştirme
        x_pts, y_pts = self.get_obstacle_points(angles, ranges)
        self.visualiser.update(x_pts, y_pts,
                                F_att=F_att, F_rep=F_rep, F_total=F_total,robot_x=0.0, robot_y=0.0
                                )

    def compute_attractive_force(self):
        # Çekici kuvvet (örnek: hedefe sürekli rastgele keşif hedefi belirlenebilir)
        # Şu an sabit ileri yön çekim
        return np.array([self.k_att, 0.0])

    def compute_repulsive_force(self, angles, ranges):
        F_rep = np.zeros(2)
        # Itici kuvvetlerin toplamını hesapla
        for ang, dist in zip(angles, ranges):
            if dist < self.rep_range:
                # itici kuvvet
                mag = self.k_rep * (1.0/dist - 1.0/self.rep_range) / (dist**2)
                F_rep += mag * np.array([-math.cos(ang), -math.sin(ang)])
        return F_rep

    def compute_twist(self, F: np.ndarray):
        # TODO: dünya→gövde dönüşümü, hız sınırlandırma vb. ekle
        # Şimdilik
        linear = F[0]
        angular = math.atan2(F[1], F[0])
        return linear, angular

    def get_obstacle_points(self, angles, ranges):
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        return xs.tolist(), ys.tolist()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()