# Ros2 gazebo for warehouse AGV/AMR

This repo aims to develop a simulation environment for warehouse AGV/AMR for study & research.

## 🚀 Project Implementation Checklist
### Core Simulation Features
- [ ] Basic AGV model in Gazebo
  - [ ] URDF/Xacro model
  - [ ] Proper inertial properties
  - [ ] Collision boundaries
  - [ ] Visual appearance

- [ ] Sensor integration
  - [ ] 2D Lidar
  - [x] 3D Camera/Depth sensor
  - [ ] Ultrasonic for dynamic obstacle avoidance
  - [x] IMU
  - [x] Odometry
- [x] Environment
  - [x] Warehouse world model
  - [x] Obstacles and racks
  - [x] Loading/unloading zones

### Navigation Stack
- [x] SLAM Implementation
  - [x] Online SLAM with slam_toolbox
  - [x] Map saving/loading functionality
- [x] Path Planning
  - [x] Global planner configuration
  - [ ] Local planner configuration
  - [ ] Costmaps setup
- [ ] Autonomous Navigation
  - [x] Waypoint following
  - [ ] Obstacle avoidance
  - [ ] Recovery behaviors
  - [ ] Charging station

### Control System
- [ ] Motion controllers
  - [x] Velocity controllers
  - [ ] PID tuning
  - [x] Keyboard Control 
  - [ ] Multiplexer velocity control

## Getting Started

### Dependencies
Ubuntu 22.04 Jammy,
ROS2 Humble,
Gazebo Classic (EOL as 2025.1)

### Installing Gazebo classic
Official website for installation: [Tutorial](https://classic.gazebosim.org/tutorials?tut=install_ubuntu) <br />

* Setup package source 
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```
* Update
```
sudo apt-get update
```
* Install
```
sudo apt-get install gazebo
sudo apt-get install libgazebo-dev
```
And also the ros2-gazebo package: [Tutorial](https://classic.gazebosim.org/tutorials?tut=ros2_installing)
```
sudo apt install ros-humble-gazebo-ros-pkgs
```

### Install Dependencies
* Pointcloud to laserscan package
```
sudo apt install ros-humble-pointcloud-to-laserscan
```

* Nav2 and Slam-toolbox
``` 
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

### Build package and run

* Build
```
cd ros2_ws2/
colcon build --symlink-install
```
* Source environment (to have access to your workspace packages)
``` 
source install/setup.bash
```

* Run example
```
ros2 launch my_agv2 agv5_demo.launch.py
```

