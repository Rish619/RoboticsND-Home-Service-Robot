#!/bin/sh
#xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=~/catkin_ws/src/worlds/u.world " &
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/rishi/VsCode_Projects/RoboticsND_Home_Service_Robot/catkin_ws_3/src/worlds/service.world " &
sleep 3
#xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=~/catkin_ws/src/worlds/umap.yaml " &
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/rishi/VsCode_Projects/RoboticsND_Home_Service_Robot/catkin_ws_3/src/world/service_map.yaml" &
sleep 3
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 3
xterm  -e  " rosrun pick_objects pick_objects "
