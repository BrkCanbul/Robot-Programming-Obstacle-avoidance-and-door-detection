#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class ObjectAvoidanceNode(Node):
    def __init__(self):
        super().__init__("object_avoidance_node")

        # Abonelikler: LIDAR verileri (/scan) ve odometri verileri (/layka_controller/odom) alınır
        self.lidar_sub = self.create_subscription(
            LaserScan, "/scan", self.lidar_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/layka_controller/odom", self.odometry_callback, 10
        )

        # Parametreler: Engel kaçınma ve hareket için
        self.declare_parameter("K_rep", 0.2)  # İtici kuvvet katsayısı
        self.declare_parameter("rep_field", 0.65)  # Engel itme alanının mesafesi
        self.declare_parameter("v_linear_max", 0.8)  # Maksimum doğrusal hız
        self.declare_parameter("v_angular_max", 1.2)  # Maksimum açısal hız
        self.declare_parameter("default_linear_speed", 0.5)  # Varsayılan doğrusal hız
        self.declare_parameter("stuck_threshold", 0.05)  # Sıkışma için hız eşik
        self.declare_parameter("stuck_time", 2.0)  # Sıkışma süresi eşik
        self.declare_parameter("min_distance", 0.3)  # Minimum engel mesafesi
        self.declare_parameter("att_strength", 0.25)  # Çekici kuvvet katsayısı

        # Parametrelerin alınması
        self.k_rep = self.get_parameter("K_rep").value
        self.rep_field = self.get_parameter("rep_field").value
        self.v_linear_max_ms = self.get_parameter("v_linear_max").value
        self.v_angular_max_rads = self.get_parameter("v_angular_max").value
        self.default_linear_speed = self.get_parameter("default_linear_speed").value
        self.stuck_threshold = self.get_parameter("stuck_threshold").value
        self.stuck_time = self.get_parameter("stuck_time").value
        self.min_distance = self.get_parameter("min_distance").value
        self.att_strength = self.get_parameter("att_strength").value

        # Hız komutları (/layka_controller/cmd_vel) için
        self.publisher = self.create_publisher(
            TwistStamped, "/layka_controller/cmd_vel", 10
        )
        # Kontrol döngüsü: 20 Hz (0.05 s) frekansta çalışır
        self.create_timer(0.05, self.control_loop)

        self.lastest_scan = None  # Son LIDAR verisi
        self.current_twist = TwistStamped()  # Son hız komutu
        self.robot_gx = 0.0  # Robotun x konumu
        self.robot_gy = 0.0  # Robotun y konumu
        self.last_position = None  # Son konum (sıkışma kontrolü için)
        self.stuck_timer = 0.0  # Sıkışma süresi
        self.previous_angles = []  # Son seçilen açılar (maksimum 5)

        self.get_logger().info("Object Avoidance Node Started")

    def lidar_callback(self, msg):
        # LIDAR verilerini kaydet
        self.lastest_scan = msg

    def odometry_callback(self, msg):
        # Robotun konumunu güncelle
        self.robot_gx = msg.pose.pose.position.x
        self.robot_gy = msg.pose.pose.position.y

    def control_loop(self):
        # Kontrol döngüsü: Her 0.05 saniyede çalışır, robotun hareketini planlar
        if self.lastest_scan is None:
            return

        scan = self.lastest_scan
        # İtici kuvvet
        F_rep = self._compute_rep_force(scan)
        # Çekici kuvvet
        angle_opening = self.find_widest_opening_angle(scan)
        # Dinamik çekici kuvvet: sıkışma eğiliminde artır
        att_strength_dynamic = self.att_strength * (1.5 if self.stuck_timer > 0.5 else 1.0)
        F_att = att_strength_dynamic * np.array(
            [np.cos(angle_opening), np.sin(angle_opening)]
        )
        # Toplam kuvvet
        F_total = F_rep + F_att

        # Sıkışma kontrolü
        if self.is_stuck():
            self.handle_stuck_situation()
        else:
            self.stuck_timer = 0.0
            # Hız komutlarını hesapla ve yayınla
            twist_command = self.compute_twist(F_total)
            self.publisher.publish(twist_command)
            self.current_twist = twist_command

    def _compute_rep_force(self, scan: LaserScan) -> np.ndarray:
        # İtici kuvvet hesaplama
        ranges = np.array(scan.ranges)  # LIDAR mesafe ölçümleri
        angle_min = scan.angle_min  # Minimum açı
        angle_inc = scan.angle_increment  # Açı artışı
        f_rep_total = np.zeros(2, dtype=np.float32)  # Toplam itici kuvvet vektörü

        # Her bir LIDAR ışını için itici kuvvet hesapla
        for index, d_i in enumerate(ranges):
            theta_i = angle_min + index * angle_inc

            # Geçersiz veya uzak mesafeleri atla
            if np.isinf(d_i) or np.isnan(d_i) or d_i > self.rep_field or d_i <= 0.0:
                continue

            # Potansiyel alanlar formülü: f_i = k_rep * (1/d_i - 1/rep_field)
            coeff = (1.0 / max(d_i, self.min_distance)) - (1.0 / self.rep_field)
            f_i = self.k_rep * coeff

            # Işının yön vektörü
            ux = np.cos(theta_i)
            uy = np.sin(theta_i)

            # İtici kuvvet engel yönüne ters
            f_rep_total += -f_i * np.array([ux, uy])

        return f_rep_total

    def find_widest_opening_angle(self, scan: LaserScan):
        ranges = np.array(scan.ranges)
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        num_ranges = len(ranges)

        # -90 derece (rad) ve +90 derece (rad) aralığındaki indeksleri bul
        start_angle = -np.pi / 2
        end_angle = np.pi / 2
        start_index = int(np.ceil((start_angle - angle_min) / angle_inc))
        end_index = int(np.floor((end_angle - angle_min) / angle_inc))

        # Sınırları aşmamak için indeksleri düzelt
        start_index = max(0, start_index)
        end_index = min(num_ranges - 1, end_index)

        # İlgili aralıktaki mesafeleri al
        selected_ranges = ranges[start_index : end_index + 1]
        valid_ranges = np.where(np.isfinite(selected_ranges) & (selected_ranges > self.min_distance), selected_ranges, 0.0)

        # Pencereleme yöntemi: Mesafeleri yumuşatarak geniş açık alanları tespit et
        window_size = 30
        if len(valid_ranges) < window_size:
            window_size = max(1, len(valid_ranges) // 2)
        smoothed_ranges = np.convolve(
            valid_ranges, np.ones(window_size) / window_size, mode="valid"
        )

        # En geniş 3 açık alanı bul
        sorted_indices = np.argsort(smoothed_ranges)[::-1][:3]  # En büyük 3 indeks
        if len(sorted_indices) == 0:
            # Hiç açık alan yoksa varsayılan açı
            max_index = np.argmax(smoothed_ranges)
            target_angle = angle_min + (start_index + max_index + window_size // 2) * angle_inc
        else:
            # Ağırlıklı rastgele seçim
            weights = np.clip(smoothed_ranges[sorted_indices], 0, None)  # Negatif değerleri sıfıra çek
            weight_sum = np.sum(weights)
            if weight_sum == 0:
                weights = np.ones_like(weights) / len(weights)  # Sıfır toplamı için eşit ağırlık
            else:
                weights = weights / weight_sum  # Normalizasyon
            chosen_index = np.random.choice(sorted_indices, p=weights)
            target_angle = angle_min + (start_index + chosen_index + window_size // 2) * angle_inc

            # Önceki açılardan kaçın
            if len(self.previous_angles) > 0 and np.random.rand() < 0.7:  # %70 ihtimalle
                for _ in range(3):  # 3 deneme
                    if any(abs(target_angle - prev_angle) < 0.3 for prev_angle in self.previous_angles):
                        chosen_index = np.random.choice(sorted_indices, p=weights)
                        target_angle = angle_min + (start_index + chosen_index + window_size // 2) * angle_inc
                    else:
                        break

        # Hedef açıyı kaydet (maksimum 5 açı sakla)
        self.previous_angles.append(target_angle)
        if len(self.previous_angles) > 5:
            self.previous_angles.pop(0)

        return target_angle

    def is_stuck(self):
        # Sıkışma kontrolü: Robotun hareketi yetersizse sıkışmış kabul edilir
        if self.last_position is None:
            self.last_position = (self.robot_gx, self.robot_gy)
            return False

        # Mevcut ve önceki konum arasındaki mesafeyi hesapla
        current_position = (self.robot_gx, self.robot_gy)
        distance_moved = np.hypot(
            current_position[0] - self.last_position[0],
            current_position[1] - self.last_position[1],
        )
        speed = distance_moved / 0.05
        self.last_position = current_position

        # Hız, eşik değerin altındaysa sıkışma sayacı artırılır
        if speed < self.stuck_threshold:
            self.stuck_timer += 0.05
            if self.stuck_timer > self.stuck_time:
                self.get_logger().info("Robot detected as stuck")
                return True
        else:
            self.stuck_timer = 0.0  # Hareket varsa sayaç sıfırlanır
        return False

    def handle_stuck_situation(self):
        # Sıkışma durumu: Robot en geniş açık alana dönerek kurtulmayı dener
        scan = self.lastest_scan
        if scan is None:
            return

        ranges = np.array(scan.ranges)
        valid_ranges = np.where(np.isfinite(ranges) & (ranges > 0.0), ranges, -1.0)

        # En geniş açık alanı bul
        window_size = 30
        smoothed_ranges = np.convolve(
            valid_ranges, np.ones(window_size) / window_size, mode="valid"
        )
        max_index = np.argmax(smoothed_ranges)
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        target_angle = angle_min + (max_index + window_size // 2) * angle_inc

        # Açık alana dön ve yavaş ilerle
        twist = TwistStamped()
        angular_speed = float(
            np.clip(
                target_angle * 0.5, -self.v_angular_max_rads, self.v_angular_max_rads
            )
        )
        twist.twist.angular.z = angular_speed
        twist.twist.linear.x = float(self.default_linear_speed * 0.5)
        twist.header.frame_id = "base_link"
        twist.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(twist)
        self.get_logger().info(
            f"Robot is stuck, turning towards widest open space at angle {target_angle:.2f} rad"
        )

    def compute_twist(self, F_total: np.ndarray):
        # Hız komutlarını hesapla: Toplam kuvvetten doğrusal ve açısal hızlar türetilir
        v_cmd = self.default_linear_speed
        w_cmd = 0.0

        # Toplam kuvvet sıfıra yakın değilse yön ve hız hesaplanır
        if np.linalg.norm(F_total) > 1e-6:
            angle_total = np.arctan2(F_total[1], F_total[0])  # Çekici ve itici kuvvet birlikte
            w_cmd = -angle_total * 0.6  # Engelden uzaklaş ve açık alana yönel
            # Doğrusal hız: Engel açısına bağlı olarak azalır
            v_cmd *= max(0.4, 1.0 - np.abs(angle_total) / (np.pi / 2))

        # Hızları sınırlandır
        v_cmd = np.clip(v_cmd, -self.v_linear_max_ms, self.v_linear_max_ms)
        w_cmd = np.clip(w_cmd, -self.v_angular_max_rads, self.v_angular_max_rads)

        # Hız komutunu hazırla ve zaman damgası ekle
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
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()