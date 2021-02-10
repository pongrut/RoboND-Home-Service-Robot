#!/bin/bash

if [[ $1 == turtlebot ]]; then
    echo "Launching turtlebot..."
    xterm -geometry 80x20+0+0 -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:="$PWD"/src/worlds/pongrut.world " &
    sleep 5
    xterm -geometry 80x20+0+200 -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:="$PWD"/src/map/pongrut_map.yaml " &
    sleep 5
    xterm -geometry 80x20+0+400 -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
else
    echo "Launching jetbot..."
    xterm  -geometry 80x20+0+0 -e  " roslaunch jetbot jetbot_world.launch world_file:="$PWD"/src/worlds/pongrut.world " &
    sleep 5
    xterm  -geometry 80x20+0+200 -e  " roslaunch jetbot jetbot_amcl.launch map_file:="$PWD"/src/map/pongrut_map.yaml " &
    sleep 5
    xterm  -geometry 80x20+0+400 -e  " roslaunch jetbot jetbot_view_navigation.launch " &

fi

sleep 10
xterm -geometry 80x20+0+600 -e "rosrun add_markers add_markers" &
sleep 5
xterm -geometry 80x20+0+800 -e "rosrun pick_objects pick_objects"

