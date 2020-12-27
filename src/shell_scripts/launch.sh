#!/bin/sh
xterm  -e  " gazebo " &
sleep 5
xterm  -e  " source /opt/ros/melodic/setup.bash; roscore" &
sleep 5
xterm  -e  " rosrun rviz rviz"