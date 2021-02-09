#!/bin/sh
#xterm  -e  " roslaunch jetbot jetbot_world.launch world_file:=/home/pongrut/catkin_ws/src/worlds/pongrut.world " &
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:="$PWD"/src/worlds/pongrut.world " &
sleep 5

#xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:="$PWD"/src/map/pongrut_map.yaml " &
xterm  -e  " roslaunch jetbot jetbot_amcl.launch map_file:="$PWD"/src/map/pongrut_map.yaml " &
sleep 5

# xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
xterm  -e  " roslaunch jetbot jetbot_view_navigation.launch " &


sleep 15
xterm -e "rosrun pick_objects pick_objects"
