# ASRL Roam Project ROS2 Implementation
Website [https://mae.ucf.edu/ASRL/robotics/]

## Description
TODO

## Maintainer
Ros2 setup created by Bastian Weiss

Email: ba379142@ucf.edu

# Setup
## Docker Image
Install Docker
[https://docs.docker.com/install/]

Install Rocker
[https://github.com/osrf/rocker]

## Build the Docker Container
```
docker image pull ros:jazzy
```
```
docker image build --no-cache -t ws_asrl_roam .
```
```
rocker --ssh --x11 --devices /dev/ttyUSB0 /dev/ttyACM1 --name asrl_roam ws_asrl_roam bash
```

## Build the Ros2 Enviroment for Gazbeo
```
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch roam_base_gazebo gazebo_model.launch.py
```
## Control The Base with Keyboard Teleop
```
docker exec -it asrl_roam bash
```
```
source /opt/ros/jazzy/setup.bash
```
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

## Build the Ros2 Enviroment for Physical Base

Step 1: Open the Arduino IDE and connect to board.

Step 2: Flash firmware

Step 3: Make sure which port you are on (ACM0 or ACM1)

Step 4: Screen and Cat the port
```
screen /dev/ttyACM1 115200
```
Press: Ctrl+A, K, Enter
```
cat /dev/ttyACM1
```
Step 5: Close Arduino IDE

Step 6: Launch Ros2 Hardware
```
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch roam_arduino_hardware real_model.launch.py
```



```
source /opt/ros/jazzy/setup.bash
cd ws_moveit
source install/setup.bash
cd
cd ws_asrl_roam
colcon build
source install/setup.bash
```

