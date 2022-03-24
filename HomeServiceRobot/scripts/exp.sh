#!/bin/sh
s_path="$PWD/../../devel/setup.bash"
xterm -e "source $s_path;roslaunch my_robot world.launch"