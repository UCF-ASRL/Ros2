# ASRL Roam Project ROS2 Implementation
Website [https://mae.ucf.edu/ASRL/robotics/]

## Description
TODO

## Maintainer
Ros2 setup created by Bastian Weiss

Email: ba379142@ucf.edu

## Setup
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
rocker --ssh --x11 ws_asrl_roam --name asrl_roam
```

## Build the Ros2 Enviroment
```
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch roam_base gazebo_model.launch.py
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
