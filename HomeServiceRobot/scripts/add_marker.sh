#!/bin/sh

c_path="../rvizConfig/nav.rviz"
s_path="$PWD/../../devel/setup.bash"

xterm -e "source $s_path;roslaunch my_robot world.launch " &

sleep 15

xterm -e "source $s_path;roslaunch my_robot amcl.launch" &

sleep 7

xterm -e "source $s_path;rviz --display-config $c_path" &

sleep 7

xterm -e "source $s_path;roslaunch add_markers markers_test.launch"
