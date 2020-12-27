#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/rishi/VsCode_Projects/RoboticsND_Home_Service_Robot/catkin_ws_3/src/worlds/service.world " &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/rishi/VsCode_Projects/RoboticsND_Home_Service_Robot/catkin_ws_3/src/worlds/service_map.yaml " &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm -e "rosrun add_markers add_markers " &
sleep 5
xterm -e "rosrun pick_objects pick_objects"