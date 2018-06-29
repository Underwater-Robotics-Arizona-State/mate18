#!/bin/bash
#runs all the scripts in a nice little package

#opens new terminal to run ROS
gnome-terminal -e /home/alexb/MATE-2017/Scripts/ros_launch.sh
sleep 3
#launchs the xbox controller
gnome-terminal -e /home/alexb/MATE-2017/Scripts/xbox_launch.sh
wait until
#inputs the data into sensors_msgs/joy.h
gnome-terminal -e /home/alexb/MATE-2017/Scripts/joystick_launch.sh

