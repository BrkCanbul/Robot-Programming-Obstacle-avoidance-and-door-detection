import time
import numpy as np
from geometry_msgs.msg import TwistStamped,Twist


class PIDController:
    def __init__(self):
        self.last_time = None
        self.last_angle_error = 0.0
        self.angle_integral = 0.0

        self.last_distance_error = 0.0
        self.distance_integral = 0.0

        self.Kp_ang = 2.5
        self.Ki_ang = 0.0
        self.Kd_ang = 0.5

        self.Kp_lin = 1.0
        self.Ki_lin = 0.0
        self.Kd_lin = 0.2

    def compute_twist(self, F: np.ndarray, robot_yaw: float, v_max: float, w_max: float) -> TwistStamped:
        twist = TwistStamped()

        current_time = time.time()
        dt = current_time - self.last_time if self.last_time else 1e-3
        self.last_time = current_time

        goal_angle = np.arctan2(F[1], F[0])
        angle_error = self._normalize_angle(goal_angle - robot_yaw)

        self.angle_integral += angle_error * dt
        angle_derivative = (angle_error - self.last_angle_error) / dt
        self.last_angle_error = angle_error

        w = (self.Kp_ang * angle_error +
             self.Ki_ang * self.angle_integral +
             self.Kd_ang * angle_derivative)

        distance = np.linalg.norm(F)
        self.distance_integral += distance * dt
        distance_derivative = (distance - self.last_distance_error) / dt
        self.last_distance_error = distance

        v = (self.Kp_lin * distance +
             self.Ki_lin * self.distance_integral +
             self.Kd_lin * distance_derivative)

        
        twist.twist.linear.x = float(np.clip(v, -v_max, v_max))
        twist.twist.angular.z = float(np.clip(w, -w_max, w_max))
        return twist

    def _normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi