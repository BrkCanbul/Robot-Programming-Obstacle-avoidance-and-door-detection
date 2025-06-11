# pid_Controller.py
import numpy as np
import math
from geometry_msgs.msg import TwistStamped, Twist
from rclpy.clock import Clock
import time as t

class PIDHistory:
    def __init__(self):
        self.desired_yaws = []
        self.actual_yaws = []
        self.w_commands = []
        self.v_commands = []
        self.timestamps = []           # zaman bilgisi için liste ekledik
    
    # timestamp argümanını ekledik
    def add_item(self, ac_yaw: float, des_yaw: float,
                 w_command: float, v_command: float, timestamp: float):
        self.actual_yaws.append(ac_yaw)
        self.desired_yaws.append(des_yaw)
        self.w_commands.append(w_command)
        self.v_commands.append(v_command)
        self.timestamps.append(timestamp)
    
    def save_history(self, time_str):
        with open(f"pid_history_{time_str}.csv", "w") as file:
            file.write("time,desired_yaw,actual_yaw,w_command,v_command\n")
            for ts, des, act, w, v in zip(
                    self.timestamps,
                    self.desired_yaws,
                    self.actual_yaws,
                    self.w_commands,
                    self.v_commands):
                file.write(f"{ts:.4f},{des:.4f},{act:.4f},{w:.4f},{v:.4f}\n")
        print("saved_to_file")

class PIDController:
    def __init__(self,**kwargs):
        """PID denetleyicisi."""
        
        # PID terms
        self.last_ang_err = 0.0
        self.int_ang = 0.0
        self.last_lin_err = 0.0
        self.int_lin = 0.0
        
        defaults = {
            'Kp_ang': 0.30, 'Ki_ang': 0.02, 'Kd_ang': 0.10,
            'Kp_lin': 0.60, 'Ki_lin': 0.01, 'Kd_lin': 0.05,
            'deadband_ang': 0.05, 'd_thresh': 0.20, 'alpha': 0.7
        }
        
        defaults.update(kwargs)
        self.Kp_ang = defaults['Kp_ang']
        self.Ki_ang = defaults['Ki_ang']
        self.Kd_ang = defaults['Kd_ang']
        self.Kp_lin = defaults['Kp_lin']
        self.Ki_lin = defaults['Ki_lin']
        self.Kd_lin = defaults['Kd_lin']
        
        # Dead-band, smoothing ve alt eşik değerleri
        self.deadband_ang = defaults["deadband_ang"]    # rad
        self.d_thresh     = defaults["d_thresh"]    # m
        self.alpha        = defaults["alpha"]     # smoothing faktor
        for key in kwargs:
            if key not in defaults:
                raise ValueError(f"Unknown parameter: {key}")
        
        

        # history
        self.history = PIDHistory()
        
        

        # smoothing için önceki komutlar
        self.w_prev = 0.0
        self.v_prev = 0.0

        # ROS saati
        self.clock = Clock()

    def normalize(self, ang):
        """–π…+π’ye sarar."""
        return math.atan2(math.sin(ang), math.cos(ang))

    def save_hist(self):
        now = t.time()
        h = int((now/3600)    % 24)
        m = int((now/60)      % 60)
        s = int( now          % 60)
        self.history.save_history(f"{h:02d}:{m:02d}:{s:02d}")
        
        
    def compute_twist_p(self,
                        F: np.ndarray,
                        robot_yaw: float,
                        v_max: float,
                        w_max: float) -> TwistStamped:
        """
        Proportional-only controller: computes a TwistStamped
        using only the proportional gains (Kp_ang, Kp_lin).
        """
        # 1) Dünya → gövde dönüşümü
        c, s = math.cos(robot_yaw), math.sin(robot_yaw)
        R    = np.array([[ c,  s],
                         [-s,  c]])
        fx, fy = R.dot(F)

        # 2) Hata hesapları
        d           = math.hypot(fx, fy)
        desired_yaw = math.atan2(fy, fx)
        ang_err     = self.normalize(desired_yaw)
        # Dead-band (küçük açılarda dönmeyi atla)
        if abs(ang_err) < self.deadband_ang:
            ang_err = 0.0
            
        lin_err = d * math.cos(ang_err)

        # 3) Sadece P-terimleri
        w_raw = self.Kp_ang * ang_err
        v_raw = self.Kp_lin * lin_err

        # 4) Sınırla
        w = -float(np.clip(w_raw, -w_max, w_max))
        v =  float(np.clip(v_raw, -v_max, v_max))

        # 5) TwistStamped paketle
        cmd = TwistStamped()
        cmd.header.stamp    = self.clock.now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist           = Twist()
        cmd.twist.linear.x  = v
        cmd.twist.angular.z = w
        
        ts = self.clock.now().seconds_nanoseconds()[0] + \
             self.clock.now().seconds_nanoseconds()[1]*1e-9
        self.history.add_item(
            ac_yaw    = robot_yaw,
            des_yaw   = desired_yaw,
            w_command = w,
            v_command = v,
            timestamp = ts
        )

        return cmd

    def compute_twist(self, F: np.ndarray, robot_yaw: float,
                      dt: float, v_max: float, w_max: float):
        # 1) Dünya→gövde dönüşümü
        c, s   = math.cos(robot_yaw), math.sin(robot_yaw)
        R      = np.array([[ c,  s],
                           [-s,  c]])
        fx, fy = R.dot(F)
        d      = math.hypot(fx, fy)

        # 2) Hata hesapları
        desired_yaw = math.atan2(fy, fx)
        
        desired_yaw_rel = math.atan2(fy, fx)
        desired_yaw_global = self.normalize(robot_yaw + desired_yaw_rel)
        
        
        
        ang_err     = self.normalize(desired_yaw)     # zaten robot eksenine göre
        # Dead-band
        if abs(ang_err) < self.deadband_ang:
            ang_err = 0.0
        if abs(ang_err) > math.pi/2:
            v_raw = 0.0
        lin_err = d * math.cos(ang_err)

        # 3) Açısal PID
        self.int_ang   += ang_err * dt
        der_ang        = (ang_err - self.last_ang_err) / dt if dt>1e-6 else 0.0
        w_raw          = (self.Kp_ang*ang_err +
                          self.Ki_ang*self.int_ang +
                          self.Kd_ang*der_ang)
        self.last_ang_err = ang_err

        # 4) Lineer PID (ancak çok yakınsa dur)
        if d < self.d_thresh:
            v_raw = 0.0
        else:
            self.int_lin   += lin_err * dt
            der_lin        = (lin_err - self.last_lin_err) / dt if dt>1e-6 else 0.0
            v_raw          = (self.Kp_lin*lin_err +
                              self.Ki_lin*self.int_lin +
                              self.Kd_lin*der_lin)
            self.last_lin_err = lin_err

        # 5) Alçak geçiren filtre (smoothing)
        w = self.alpha*self.w_prev + (1-self.alpha)*w_raw
        v = self.alpha*self.v_prev + (1-self.alpha)*v_raw
        self.w_prev, self.v_prev = w, v

        # 6) Sınırla
        w = -float(np.clip(w, -w_max, w_max))
        v = float(np.clip(v, -v_max, v_max))
        
        # log values 
        print(f"PIDController: v={v:.2f}, w={w:.2f}, "
              f"desired_yaw={desired_yaw_global:.2f}, "
              f"robot_yaw={robot_yaw:.2f}, "
              f"ang_err={ang_err:.2f}, lin_err={lin_err:.2f}")
            
        # 7) TwistStamped paketle
        cmd = TwistStamped()
        cmd.header.stamp     = self.clock.now().to_msg()
        cmd.header.frame_id  = 'base_link'
        cmd.twist            = Twist()
        cmd.twist.linear.x   = v
        cmd.twist.angular.z  = w

        # 8) Geçmiş kaydı
        ts = self.clock.now().seconds_nanoseconds()[0] + \
             self.clock.now().seconds_nanoseconds()[1]*1e-9
        self.history.add_item(
            ac_yaw    = robot_yaw,
            des_yaw   = desired_yaw,
            w_command = w,
            v_command = v,
            timestamp = ts
        )
        return cmd
