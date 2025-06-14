
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
GOAL_Y=0.0
K_ATT=1.1
K_REP=1.5
D0=10.0
V_MAX=0.3
W_MAX=0.8
MIN_DIST=0.01
MAX_REP=1.0


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


ros2 launch layka_description layka_odev_empty.launch.py >> /dev/null 2>&1 &
echo "Gazebo başlatiliyor"

sleep 4
ros2 launch layka_controller  controller.launch.py >> /dev/null 2>&1 &
echo "kontrolcü basladi"

ros2 run layka_examples avoid_obstacles --ros-args \
        -p goal_x:=${GOAL_X}        \
        -p goal_y:=${GOAL_Y}        \
        -p k_att:=${K_ATT}          \
        -p k_rep:=${K_REP}          \
        -p d0:=${D0}                \
        -p v_linear_max:=${V_MAX}          \
        -p w_angular_max:=${W_MAX}          \
        -p d0:=${D0}                \
        -p v_linear_max:=${V_MAX}   \
        -p w_angular_max:=${W_MAX}  \
        -p min_distance:=${MIN_DIST}\
        -p max_rep:=${MAX_REP}      \
        -p Kp_ang:=${KP_ANG}        \
        -p Ki_ang:=${KI_ANG}        \
        -p Kd_ang:=${KD_ANG}        \
        -p Kp_lin:=${KP_LIN}        \
        -p Ki_lin:=${KI_LIN}        \
        -p Kd_lin:=${KD_LIN}        \
        -p deadband_ang:=${DEADBAND_ANG} \
        -p d_thresh:=${D_THRESH}        \
        -p alpha:=${ALPHA}             \
        



      