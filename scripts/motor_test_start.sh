#!/bin/bash
#runs all the scripts in a nice little package

#launchs the xbox controller
sudo gnome-terminal --working-directory=/home/nasgrds/catkin_ws/scripts -e './xbox_launch.sh'
wait until
#inputs the data into sensors_msgs/joy.h
#sudo gnome-terminal --working-directory=/home/nasgrds/catkin_ws/scripts/ -e '. joystick_launch.sh'

gnome-terminal --working-directory=/home/nasgrds/catkin_ws/scripts/ -e 'roslaunch control all_launch.launch'
