

ros2 launch layka_description layka_odev_house.launch.py >> /dev/null 2>&1 &
echo "Gazebo başlatiliyor"

sleep 4
ros2 launch layka_controller  controller.launch.py >> /dev/null 2>&1 &
echo "kontrolcü basladi"

ros2 run layka_examples avoid_obstacles 