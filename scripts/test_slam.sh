#!/bin/sh

if [[ $1 == turtlebot ]]; then
    echo "Launching turtlebot..."
    xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch " &
    sleep 5
    xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch " &
    sleep 5
    xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
    sleep 5
    xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch "
else
    echo "Launching jetbot..."
    xterm  -e   " roslaunch jetbot jetbot_world.launch world_file:="$PWD"/src/worlds/pongrut.world " &
    sleep 5
    xterm  -e  " roslaunch jetbot jetbot_gmapping.launch " &
    sleep 5
    xterm  -e  " roslaunch jetbot jetbot_view_navigation.launch " &
    sleep 5
    xterm  -e  " roslaunch jetbot teleop.launch "
fi
