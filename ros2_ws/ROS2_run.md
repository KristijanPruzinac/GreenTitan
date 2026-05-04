source /opt/ros/jazzy/setup.bash

cd /home/kristijan/Documents/GitHub/GreenTitan/ros2_ws
colcon build --packages-select robot_package
source install/setup.bash
ros2 launch robot_package robot.launch.xml

