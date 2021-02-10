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
    ├── CMakeLists.txt                                  # Link libraries
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
    │   │   ├── launch                                  # Start RTAB-Map in mapping mode
    │   │   ├── maps                                    # 
    │   │   ├── meshes                                  # 
    │   │   ├── rviz                                    # 
    │   │   ├── urdf                                    #       
    │   │   └── worlds                                  #    
    ├── map                                             # map files
    │   ├── localization.launch                         # Start RTAB-Map in localization mode
    │   ├── mapping.launch                              # Start RTAB-Map in mapping mode
    │   ├── multisession_mapping.launch                 # Start RTAB-Map in mulisession mapping mode
    │   ├── navigation.launch                           # Start the move_base navigation
    │   ├── robot_description.launch                    # Robot URDF description
    │   ├── teleop.launch                               # Start the teleop package
    │   └── world.launch                                # Initialize robot in Gazebo environment
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
    └── worlds                                          # simulated world in Gazebo
        └── pongrut.world                               # custom world file of project
```
