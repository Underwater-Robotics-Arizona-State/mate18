gnome-terminal --tab --title="Xbox launch" --working-directory="/" -e "bash -c 'xbox_launch.sh';bash"
wait until

gnome-terminal --tab --title="ROS master launch" --working-directory="/" -e "bash -c 'roslaunch control master_launch.launch';bash"