#!/bin/bash
#runs all the scripts in a nice little package

#launchs the xbox controller
sudo gnome-terminal --working-directory=/home/nasgrds/catkin_ws/scripts -e './xbox_launch.sh'
wait until
sudo gnome-terminal --working-directory=/home/nasgrds/catkin_ws/ -e 'roslaunch control all_launch.launch'
