import matplotlib.pyplot as plt
import numpy as np

class Visualizer:
    def __init__(self):
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 8))

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
        self.ax.plot(0, 0, 'ro', label=f'Robot {robot_x:.2f},{robot_y:.2f}')
        self.ax.scatter(x_points, y_points, c='b', s=10, label='Lidar')
        # Kuvvet vektörleri
        if F_att is not None:
            self.ax.quiver(0, 0, F_att[0]/10, F_att[1]/10, angles='xy', scale_units='xy',
                           scale=1, color='g', label=f'F_att (çekici): ({F_att[0]:.2f},{F_att[1]:.2f})')
        if F_rep is not None:
            self.ax.quiver(0, 0, F_rep[0]/10, F_rep[1]/10, angles='xy', scale_units='xy',
                           scale=1, color='r', label=f'F_rep (tarakitici) ({F_rep[0]:.2f},{F_rep[1]:.2f})')
        if F_total is not None:
            self.ax.quiver(0, 0, F_total[0]/10, F_total[1]/10, angles='xy', scale_units='xy',
                           scale=1, color='k', label=f'F_total ({F_total[0]:.2f},{F_total[1]:.2f})')

        self.ax.legend(loc='upper right')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)
