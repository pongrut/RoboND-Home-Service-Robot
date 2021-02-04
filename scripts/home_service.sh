#!/bin/sh
#xterm  -e  " roslaunch jetbot jetbot_world.launch world_file:=/home/pongrut/catkin_ws/src/worlds/pongrut.world " &
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/pongrut/catkin_ws/src/worlds/pongrut.world " &
#xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5

#xterm  -e  " roslaunch jetbot jetbot_amcl.launch map_file:=/home/pongrut/catkin_ws/src/worlds/pongrut_map.yaml " &
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/pongrut/catkin_ws/src/worlds/pongrut_map.yaml " &
#xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch " &

sleep 5
#xterm  -e  " roslaunch jetbot jetbot_view_navigation.launch "
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &

sleep 15
xterm -e "rosrun add_markers add_markers" &

sleep 5
xterm -e "rosrun pick_objects pick_objects"

