#!/bin/bash

source devel/setup.bash

catkin_make clean

catkin_make

roslaunch track_node track.launch
