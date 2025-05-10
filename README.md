# Ros2 gazebo for warehouse AGV/AMR

This repo aims to develop a simulation environment for warehouse AGV/AMR for study & research.

## 🚀 Project Implementation Checklist
### Core Simulation Features
- [x] Basic AGV model in Gazebo
  - [x] URDF/Xacro model
  - [ ] Proper inertial properties
  - [x] Collision boundaries
  - [x] Visual appearance

- [ ] Sensor integration
  - [x] 3D Camera/Depth sensor
  - [x] 3D front lidar
  - [ ] 3D back lidar 
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
  - [x] Recovery behaviors
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
git clone git@github.com:erfantbtb/warehouse_amr_robot.git
cd warehouse_amr_robot/
colcon build --symlink-install
```
* Source environment (to have access to your workspace packages)
``` 
source /opt/ros/humble/setup.bash
source install/setup.bash
```

* Run example
```
ros2 launch robot_description display.launch.py
```

