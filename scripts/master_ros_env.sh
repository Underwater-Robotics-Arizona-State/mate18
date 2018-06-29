#!/bin/sh
. /opt/ros/kinetic/setup.bash
. /home/nasgrds/catkin_ws/devel/setup.bash
export ROS_IP=nasgrds-Inspiron-5570
export ROS_HOSTNAME=nasgrds-Inspiron-5570
export ROS_MASTER_URI=http://nasgrds-Inspiron-5570:11311/
export PYTHONPATH=$PYTHONPATH:$ROS_ROOT/core/roslib/src
exec "$@"
