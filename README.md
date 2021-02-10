# RoboND-Home-Service-Robot
 The simulation of a full home service robot capable of navigating to pick up and deliver virtual objects. 

[![Demo_Video](/videos/RoboND-Robot-Where-Am-I_3.gif)](https://youtu.be/ekG1Bm4HrC4)
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
.Udacity-Robotics-SLAM                      # Map My World Project
├── README.md
├── documentation                           # Project documentation
│   ├── database.png
│   ├── frames.png
│   ├── gazebo.png
│   ├── rosgraph.png
│   ├── rtab-map.png
│   ├── rviz.png
│   └── video.png
└── my_robot                                # my_robot node
    ├── CMakeLists.txt                      # Link libraries
    ├── config                              # move_base config files
    │   ├── base_local_planner_params.yaml
    │   ├── costmap_common_params.yaml
    │   ├── global_costmap_params.yaml
    │   └── local_costmap_params.yaml
    ├── database                            # Recorded database of the map
    │   └── rtabmap.db
    ├── launch
    │   ├── localization.launch             # Start RTAB-Map in localization mode
    │   ├── mapping.launch                  # Start RTAB-Map in mapping mode
    │   ├── multisession_mapping.launch     # Start RTAB-Map in mulisession mapping mode
    │   ├── navigation.launch               # Start the move_base navigation
    │   ├── robot_description.launch        # Robot URDF description
    │   ├── teleop.launch                   # Start the teleop package
    │   └── world.launch                    # Initialize robot in Gazebo environment
    ├── meshes                              # Custom robot meshes
    │   ├── chassis.dae
    │   ├── chassis.SLDPRT
    │   ├── chassis.STEP
    │   ├── hokuyo.dae
    │   ├── wheel.dae
    │   ├── wheel.SLDPRT
    │   └── wheel.STEP
    ├── package.xml
    ├── rviz                                # RViz config file for the project
    │   └── mapping.rviz
    ├── urdf                                # Robot URDF description
    │   ├── my_robot.gazebo
    │   └── my_robot.xacro
    └── worlds                              # Simulated world in Gazebo
        ├── empty.world
        └── MyWorld.world
```
