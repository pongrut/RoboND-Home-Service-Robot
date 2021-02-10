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
tree
RoboND-Home-Service-Robot                   # Home Service Robot Project
├── README.md
├── images                                  # Project documentation
│   ├── database.png
│   ├── frames.png
│   ├── gazebo.png
│   ├── rosgraph.png
│   ├── rtab-map.png
│   ├── rviz.png
│   └── video.png
├── videos.                                 # Project documentation
└── src.                                    # ROS packages
    ├── CMakeLists.txt                      # Link libraries
    ├── add_makers                          # move_base config files
    │   ├── base_local_planner_params.yaml
    │   └── local_costmap_params.yaml
    ├── jetbot                              # Recorded database of the map
    │   └── rtabmap.db
    ├── map
    │   ├── localization.launch             # Start RTAB-Map in localization mode
    │   ├── mapping.launch                  # Start RTAB-Map in mapping mode
    │   ├── multisession_mapping.launch     # Start RTAB-Map in mulisession mapping mode
    │   ├── navigation.launch               # Start the move_base navigation
    │   ├── robot_description.launch        # Robot URDF description
    │   ├── teleop.launch                   # Start the teleop package
    │   └── world.launch                    # Initialize robot in Gazebo environment
    ├── pick_objects                        # Custom robot meshes
    │   ├── chassis.dae
    │   ├── chassis.SLDPRT
    │   ├── chassis.STEP
    │   ├── hokuyo.dae
    │   ├── wheel.dae
    │   ├── wheel.SLDPRT
    │   └── wheel.STEP
    ├── rvizConfig                          # RViz config file for the project
    │   └── mapping.rviz
    ├── scripts                             # Shell scripts
    │   ├── my_robot.gazebo
    │   └── my_robot.xacro
    ├── slam_gmapping                       # slam_gmapping package
    │   ├── my_robot.gazebo
    │   └── my_robot.xacro
    ├── teleop_twist_keyboard               # teleop_twist_keyboard package
    │   ├── my_robot.gazebo
    │   └── my_robot.xacro
    ├── turtlebot                           # Turtlebot keyboard_teleop 
    │   ├── my_robot.gazebo
    │   └── my_robot.xacro    
    ├── turtlebot_interactions              # Turtlebot view_navigation.launch file 
    │   ├── my_robot.gazebo
    │   └── my_robot.xacro    
    ├── turtlebot_simulator                 # Turtlebot gazebo simluator
    │   ├── my_robot.gazebo
    │   └── my_robot.xacro    
    └── worlds                              # Simulated world in Gazebo
        ├── empty.world
        └── MyWorld.world
```
