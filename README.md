# RoboND-Home-Service-Robot
 The simulation of a full home service robot capable of navigating to pick up and deliver virtual objects [Video Clip](https://youtu.be/9t6gXnkddTM). 

[![Demo_Video](/videos/RobotND-Home-Service-Robot.gif)](https://youtu.be/9t6gXnkddTM)
![Jetbot_Model2](images/jetbot1_small.png)  


## Overview  
The simulation starts by showing the marker object at the pickup zone then the Jetbot is assigned the pickup zone as the goal.
Jetbot calculates global path planning and local path planning along the way to the goal when Jetbot has arrived at the pickup zone
and then hides the marker. After that, wait 5 seconds and assign the new goal at drop off location Jetbot calculate the destination's path, then show the marker at the drop off zone once jetbot reaches it.
## Prerequisites/Dependencies  
* Gazebo >= 7.0  
* ROS Kinetic [Ubuntu install of ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* ROS gazebo package
* ROS navigation package  
* ROS map-server package 
* ROS move-base package 
* ROS amcl package 
* ROS teleop_twist_keyboard
* ROS turtlebot package 
```
sudo apt-get update && apt-get upgrade
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control  ros-kinetic-effort-controllers
sudo apt-get install ros-kinetic-navigation
sudo apt-get install ros-kinetic-map-server
sudo apt-get install ros-kinetic-move-base
sudo apt-get install ros-kinetic-amcl
sudo apt-get install ros-kinetic-teleop-twist-keyboard
sudo apt-get install ros-kinetic-turtlebot*
```
## Create Catkin Workspace:
```
mkdir -p /home/workspace/catkin_ws/src
cd /home/workspace/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
source devel/setup.bash

cd ..
git clone https://github.com/pongrut/RoboND-Home-Service-Robot.git
cp -a RoboND-Home-Service-Robot/src/* catkin_ws/src
cd /home/workspace/catkin_ws
catkin_make
source devel/setup.bash
chmod +x src/scripts/*
chmod +x src/teleop_twist_keyboard/teleop_twist_keyboard.py
rosdep -i install gmapping turtlebot_teleop turtlebot_simulator
```

## Run the project  
* SLAM environment mapping test with test_slam.sh shell script  
```
cd /home/workspace/catkin_ws/
source devel/setup.bash

# Run with my Jetbot
./src/scripts/test_slam.sh

# Or run with Turtlebot
./src/scripts/test_slam.sh turtlebot
``` 

* Environment navigation test with test_navigation.sh shell script  
```
cd /home/workspace/catkin_ws/
source devel/setup.bash

# Run with my Jetbot
./src/scripts/test_navigation.sh

# Or run with Turtlebot
./src/scripts/test_navigation.sh turtlebot
``` 

* 2 navigation goals test with pick_objects.sh shell script 
```
cd /home/workspace/catkin_ws/
source devel/setup.bash

# Run with my Jetbot
./src/scripts/pick_objects.sh

# Or run with Turtlebot
./src/scripts/pick_objects.sh turtlebot
``` 

* Virtual object simulation test with add_marker.sh shell script 
```
cd /home/workspace/catkin_ws/
source devel/setup.bash

# Run with my Jetbot
./src/scripts/add_markers.sh

# Or run with Turtlebot
./src/scripts/add_markers.sh turtlebot
``` 

* Launch Home Service Robot with home_service.sh shell script  
```
cd /home/workspace/catkin_ws/
source devel/setup.bash

# Run Home Service with my Jetbot
./src/scripts/home_service.sh

# Or run Home Service with Turtlebot
./src/scripts/home_service.sh turtlebot
``` 
![Gazebo Screen](images/home_service_robot_3.jpg)  
Home Service Robot in Gazebo Screen

![Rviz Screen](images/home_service_robot.jpg)  
Home Service Robot in Rviz Screen
### Project structure:
```bash

.RoboND-Home-Service-Robot                              # Home Service Robot Project
├── README.md
├── writeup.md                                          # Project documentation
├── images  
│   ├── home_service_robot.jpg
│   ├── home_service_robot_2.jpg
│   ├── home_service_robot_3.jpg
│   ├── jetbot1.png
│   ├── jetbot1_small.png
│   ├── pongrut_map.jpg
│   ├── robot_tf_urdf.jpg
│   ├── rosgraph_active.png
│   ├── world.jpg
│   └── world_robot_map.jpg
├── videos.  
    └── RobotND-Home-Service-Robot.gif 
└── src.                                                # ROS packages
    ├── add_makers                                      # move_base config files
    │   ├── src
    │   │   ├── add_markers.cpp                         # add_makers node c++ source code
    │   │   └── add_markers_test.cpp                    # add_makers_test node c++ source code
    │   ├── CMakeLists.txt                              # compiler instructions
    │   └── package.xml                                 # package info
    ├── jetbot                                          # jetbot robot package
    │   │   ├── config                                  # navigation configuration files
    │   │   │   ├── costmap_common_params.yaml          # store common local & global costmap parameters
    │   │   │   ├── dwa_local_planner_params.yaml       # store parameters of the dwa_local_planner 
    │   │   │   ├── global_costmap_params.yaml          # store common global costmap parameters 
    │   │   │   ├── global_planner_params.yaml          # store common global planner parameters
    │   │   │   ├── local_costmap_params.yaml           # store common local costmap parameters      
    │   │   │   ├── move_base_params.yaml               # store move base node parameters  
    │   │   │   └── navfn_global_planner_params.yaml    # store navfn global planner parameters      
    │   │   ├── launch                                  # jetbot launch files
    │   │   │   ├── jetbot_amcl.launch                  # map_server, amcl, move_base nodes launch file
    │   │   │   ├── jetbot_gmapping.launch              # gmapping node & parameters launch file
    │   │   │   ├── jetbot_view_navigation.launch       # rviz launch file
    │   │   │   ├── jetbot_world.launch                 # robot urdf_spawner node launch file
    │   │   │   ├── robot_description.launch            # joint_state_publisher, robot_state_publisher nodes launch file
    │   │   │   ├── teleop.launch                       # teleop_twist_keyboard node launch file
    │   │   │   └── world.launch                        # default robot urdf_spawner node launch file
    │   │   ├── maps                                    # jetbot embeded map files
    │   │   │   ├── pongrut_map.pgm                     # image map file 
    │   │   │   └── pongrut_map.yaml                    # map description file
    │   │   ├── meshes                                  # mesh files
    │   │   │   ├── hokuyo.dae
    │   │   │   ├── jetbot-chassis.dae
    │   │   │   ├── jetbot-chassis.stl
    │   │   │   ├── jetbot-chassis_no_board.stl
    │   │   │   ├── jetbot-chassis_w_caster.stl
    │   │   │   ├── jetbot-left-wheel.stl
    │   │   │   ├── jetbot-right-wheel.stl   
    │   │   │   ├── jetbot-wheel.stl
    │   │   │   ├── jetson_nano.stl
    │   │   │   └── raspbery_pi_cam.stl
    │   │   ├── rviz                                    # rviz config file for jetbot
    │   │   │   └── default.rviz  
    │   │   ├── urdf                                    # robot description files   
    │   │   │   ├── jetbot.gazebo                       # gazebo sensors plugin file  
    │   │   │   └── jetbot.xacro                        # jetbot Unified Robot Description Format (URDF) file  
    │   │   └── worlds                                  # jetbot embeded world files    
    │   │   │   ├── empty.world                           
    │   │   │   └── pongrut.world                        
    │   │   ├── CMakeLists.txt                          # compiler instructions  
    │   │   └── package.xml                             # package info
    ├── map                                             # project map files
    │   ├── pongrut_map.pgm                             # image map file 
    │   └── pongrut_map.yaml                            # map description file    
    ├── pick_objects                                    # Custom robot meshes
    │   ├── src
    │   │   └── pick_objects.cpp                        # pick_objects node c++ source code
    │   ├── CMakeLists.txt                              # compiler instructions
    │   └── package.xml                                 # package info
    ├── rvizConfig                                      # RViz config file for the project
    │   └── navigation.rviz
    ├── scripts                                         # Shell scripts
    │   ├── add_markers.sh                              # test add & remove markers script
    │   ├── home_service.sh                             # home service robot project script
    │   ├── launch.sh                                   # test gazebo, ros, and rviz launch script 
    │   ├── pick_objects.sh                             # test robot automatic goals navigation script
    │   ├── test_navigation.sh                          # test manual goal setting navigation script
    │   ├── test_slam.sh                                # test manual keyboard navigation script
    │   └── urdf_tutorial.sh                            # urdf robot visualizing script   
    ├── slam_gmapping                                   # slam_gmapping package directory
    ├── teleop_twist_keyboard                           # teleop_twist_keyboard package directory
    ├── turtlebot                                       # turtlebot keyboard_teleop package directory
    ├── turtlebot_interactions                          # turtlebot view_navigation.launch package directory   
    ├── turtlebot_simulator                             # turtlebot gazebo simluator package directory
    ├── worlds                                          # simulated world in Gazebo
    │   └── pongrut.world                               # custom world file of project
    └── CMakeLists.txt                                  # Link libraries        
```
