#sudo etherwake -i enp1s0 c0:3f:d5:6b:d6:85
sudo etherwake -i enp1s0 00:c0:08:92:45:a0

source /opt/ros/kinetic/setup.bash
source /home/nasgrds/catkin_ws/devel/setup.bash
export ROS_IP=nasgrds-Inspiron-5570
export ROS_HOSTNAME=nasgrds-Inspiron-5570
export ROS_MASTER_URI=http://nasgrds-Inspiron-5570:11311/

sleep 20

roslaunch control surface.launch
