#!/bin/sh

c_path="../rvizConfig/test_slam.rviz"
s_path="$PWD/../../devel/setup.bash"
xterm -e "source $s_path;roslaunch my_robot world.launch " &

sleep 15

xterm -e "source $s_path;roslaunch my_robot slam_gmapping.launch" &

sleep 5

xterm -e "source $s_path;rviz --display-config $c_path " &

sleep 5

xterm -e "source $s_path;rosrun teleop_twist_keyboard teleop_twist_keyboard.py"
