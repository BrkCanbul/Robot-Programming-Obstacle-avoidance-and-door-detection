Change directory to layka_ws

rm -rf build log
colcon build


rosdep install --from-paths . -i -y

source install/setup.bash   (Her terminal penceresi için yapılmalıdır)

ros2 launch layka_description layka_odev_house.launch.py
ros2 launch layka_controller controller.launch.py

cd src/layka_examples/layka_examples/
python3 avoid_obstacles.py

Hata alırsanız:

- Kopyaladığınız patikada boşluk karakteri olmadığından emin olun ("/home/levent/otonom robotlar" olmaz, "/home/levent/otonom_robotlar" olmalı).

- Aşağıdaki gereksinimleri tek tek kurmanız gerekebilir:

sudo apt install ros-jazzy-joint-state-broadcaster ros-jazzy-joint-state-controller ros-jazzy-velocity-controllers
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2controlcli
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-gz-ros2-control ros-jazzy-ros-gz-bridge
sudo apt install ros-jazzy-diff-drive-controller
sudo apt-get install ros-jazzy-rplidar-ros


Diğer:

ros2 topic pub -r 10 /layka_controller/cmd_vel geometry_msgs/msg/TwistStamped \'h     (taba basacaksın)


Eğer robotu Empty World'de başlatmak isterseniz şu komutu çalıştırmalısınız:
ros2 launch layka_description layka_odev_empty.launch.py