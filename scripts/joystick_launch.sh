#!/bin/bash

rosparam set joy_node/dev /dev/input/js0

rosrun joy joy_node

#to display values in terminal use
#rostopic echo joy


