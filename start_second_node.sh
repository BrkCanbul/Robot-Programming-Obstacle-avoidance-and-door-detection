
source ./install/setup.zsh

        # self.declare_parameter("goal_x",10.0)
        # self.declare_parameter("goal_y",0.0)
        # self.declare_parameter("k_att",1.3)
        # self.declare_parameter("k_rep",0.4)
        # self.declare_parameter("d0",3.50)
        # self.declare_parameter("v_linear_max",0.2)
        # self.declare_parameter("w_angular_max",0.8)
        # self.declare_parameter("min_distance",0.01)
        # self.declare_parameter("max_rep",5.0)



GOAL_X=10.0
GOAL_Y=7.0
K_ATT=1.8
K_REP=0.2
D0=1.5
V_MAX=0.4
W_MAX=0.8
MIN_DIST=0.01
MAX_REP=10.0


# PID PARAMETERS
  # self.declare_parameter("Kp_ang",0.30)
        # self.declare_parameter("Ki_ang",0.02)
        # self.declare_parameter("Kd_ang",0.10)
        # self.declare_parameter("Kp_lin",0.60)
        # self.declare_parameter("Ki_lin",0.01)
        # self.declare_parameter("Kd_lin",0.05)
        # self.declare_parameter("deadband_ang",0.05)
        # self.declare_parameter("d_thresh",0.20)
        # self.declare_parameter("alpha",0.7)

KP_ANG=0.34            
KI_ANG=0.02
KD_ANG=0.10
KP_LIN=0.60
KI_LIN=0.01
KD_LIN=0.05
DEADBAND_ANG=0.05
D_THRESH=0.2
ALPHA=0.7


ros2 launch layka_description layka_odev_house.launch.py >> /dev/null 2>&1 &
echo "Gazebo başlatiliyor"

sleep 4
ros2 launch layka_controller  controller.launch.py >> /dev/null 2>&1 &
echo "kontrolcü basladi"

ros2 run layka_examples second_node
        # self.declare_parameter("K_att",1.0)         ## attractive coef 
        # self.declare_parameter("K_rep",1.0)         ## repulsive  coef
        # self.declare_parameter("rep_field",1.0)     ## repulsive force affection field
        # self.declare_parameter("v_linear_max",0.5)  ## m/s
        # self.declare_parameter("v_angular_max",1.0) ## rad/s
        
        # self.declare_parameter("x_goal",5.0)
        # self.declare_parameter("y_goal",6.0) 
        



      
