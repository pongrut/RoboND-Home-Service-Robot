#!/bin/sh

#xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch " &
xterm  -e   " roslaunch jetbot jetbot_world.launch world_file:="$PWD"/src/map/pongrut.world " &
sleep 5

#xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:="$PWD"/src/map/pongrut_map.yaml " &
xterm  -e   " roslaunch jetbot jetbot_amcl.launch map_file:="$PWD"/src/map/pongrut_map.yaml " &
sleep 5

#xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch "
xterm  -e  " roslaunch jetbot jetbot_view_navigation.launch "
