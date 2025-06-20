import matplotlib.pyplot as plt
import numpy as np

class Visualizer:
    def __init__(self):
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(4, 4))

        # Eksen sınırlarını KİLİTLE
        self.xlim = (-4, 4)
        self.ylim = (-4, 4)

        self.ax.set_xlim(self.xlim)
        self.ax.set_ylim(self.ylim)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.grid(True)
        self.ax.set_title("Robot Merkezli Lidar + Kuvvet Vektörleri")

    def update(self, x_points, y_points, F_att=None, F_rep=None, F_total=None ,robot_x=None,robot_y=None):
        self.ax.cla()

        # Eksen sınırlarını her çizimde ZORLA yeniden ayarla
        
        self.ax.set_xlim(self.xlim)
        self.ax.set_ylim(self.ylim)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.grid(True)

        # Robot ve lidar
        self.ax.plot(0, 0, 'r^', label=f'Robot {robot_x:.2f},{robot_y:.2f}')
        self.ax.scatter(x_points, y_points, c='b', s=10, label='Lidar')
        # Kuvvet vektörleri
        if F_att is not None:
            self.ax.quiver(0, 0, F_att[0], F_att[1], angles='xy', scale_units='xy',
                           scale=1, color='g', label=f'F_att (çekici): ({F_att[0]:.2f},{F_att[1]:.2f})')
        if F_rep is not None:
            self.ax.quiver(0, 0, F_rep[0], F_rep[1], angles='xy', scale_units='xy',
                           scale=1, color='r', label=f'F_rep (tarakitici) ({F_rep[0]:.2f},{F_rep[1]:.2f})')
        if F_total is not None:
            self.ax.quiver(0, 0, F_total[0], F_total[1], angles='xy', scale_units='xy',
                           scale=1, color='k', label=f'F_total ({F_total[0]:.2f},{F_total[1]:.2f})')

        self.ax.legend(loc='upper right')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

class VisualiserWorld:
    def __init__(self,window_size=8):
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(4, 4))
        self.window_size = window_size  # Görselleştirme penceresinin boyutu (x ve y eksenleri için)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.grid(True)
        self.ax.set_title("Dünya Merkezli Lidar + Kuvvet Vektörleri")

    def update(self, robot_x, robot_y, robot_yaw,
               x_points:np.ndarray, y_points:np.ndarray,
               F_att=None, F_rep=None, F_total=None):

        self.ax.cla()

        # Eksen sınırlarını robotun dünya koordinatlarındaki konumuna göre dinamik olarak ayarla
        self.ax.set_xlim((robot_x - self.window_size / 2, robot_x + self.window_size / 2))
        self.ax.set_ylim((robot_y - self.window_size / 2, robot_y + self.window_size / 2))
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.grid(True)

        # Robotu dünya koordinatlarında çiz
        self.ax.plot(robot_x, robot_y, 'ro', label=f'Robot ({robot_x:.2f},{robot_y:.2f})')

        # Lidar noktalarını robot merkezliden dünya koordinatlarına çevir
        x_points_world = np.array(x_points) + robot_y
        y_points_world = np.array(y_points) + robot_x
        self.ax.scatter(x_points_world, y_points_world, c='b', s=10, label='Lidar')

        # Kuvvet vektörlerini ters çevir (negatif yap) ve robotun dünya koordinatlarındaki konumundan başlat
        if F_att is not None:
            F_att = -np.array(F_att)  # Ters çevir
            self.ax.quiver(robot_x, robot_y, F_att[0], F_att[1], angles='xy', scale_units='xy',
                           scale=1, color='g', label=f'F_att (çekici): ({F_att[0]:.2f},{F_att[1]:.2f})')
        if F_rep is not None:
            F_rep = -np.array(F_rep)  # Ters çevir
            self.ax.quiver(robot_x, robot_y, F_rep[0], F_rep[1], angles='xy', scale_units='xy',
                           scale=1, color='r', label=f'F_rep (itici): ({F_rep[0]:.2f},{F_rep[1]:.2f})')
        if F_total is not None:
            F_total = -np.array(F_total)  # Ters çevir
            self.ax.quiver(robot_x, robot_y, F_total[0], F_total[1], angles='xy', scale_units='xy',
                           scale=1, color='k', label=f'F_total: ({F_total[0]:.2f},{F_total[1]:.2f})')

        self.ax.legend(loc='upper right')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)
        