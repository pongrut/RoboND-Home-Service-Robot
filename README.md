# RoboND-Home-Service-Robot
 The simulation of a full home service robot capable of navigating to pick up and deliver virtual objects. 

[![Demo_Video](/videos/RobotND-Home-Service-Robot.gif)](https://youtu.be/9t6gXnkddTM)
![Jetbot_Model2](images/jetbot1_small.png)  
![Screen Shot1](images/amcl_screen_shot01.jpg) 
![Screen Shot2](images/amcl_screen_shot02.jpg) 
![Screen Shot3](images/amcl_screen_shot03.jpg) 
![Screen Shot4](images/amcl_screen_shot04.jpg) 
## Overview  
In this project implement ROS AMCL package to accurately localize a Jetbot inside a map in the Gazebo simulation environments.
However, Jetbot model is very small can be driven **maximum at 0.l speed**.

### Project structure:
```bash

.RoboND-Home-Service-Robot                   # Home Service Robot Project
├── README.md
├── WRITEUP.md                               # Project documentation
├── images                                  
│   ├── jetbot1.png
│   └── jetbot1_small.png
├── videos.  
    └── RobotND-Home-Service-Robot.gif 
└── src.                                                # ROS packages
    ├── add_makers                                      # move_base config files
    │   ├── src
    │   │   └── add_markers.cpp                         # add_makers node c++ source code
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
    │   └── test_slam.sh                                # test manual keyboard navigation script
    ├── slam_gmapping                                   # slam_gmapping package directory
    ├── teleop_twist_keyboard                           # teleop_twist_keyboard package directory
    ├── turtlebot                                       # turtlebot keyboard_teleop package directory
    ├── turtlebot_interactions                          # turtlebot view_navigation.launch package directory   
    ├── turtlebot_simulator                             # turtlebot gazebo simluator package directory
    ├── worlds                                          # simulated world in Gazebo
        └── pongrut.world                               # custom world file of project
    └── CMakeLists.txt                                  # Link libraries        
```
