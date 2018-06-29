Since I'm having trouble getting my launch scripts to work, you'll need to follow these steps.

1) Power up everything, connect ethernet from this pc to the udoo, and plug in the xbox controller.

2) Open a terminal and type the commands below:
     cd ~/catkin_ws/scripts
     . xbox_launch.sh
   Then minimize that terminal.

3) Open a new terminal and type the following command:
     roslaunch control master_launch.launch
   Then minimize the terminal.

##########OLD##########################################################################
4) Open a new terminal and type the following command:
     ssh nagrrs@nagrrs
   Then enter the password "mate18" and type the following commands:
     rosrun rosserial_python serial_node.py /dev/ttyUSB0
   Then minimize the terminal.

5) Open a new terminal and type the following command:
     ssh nagrrs@nagrrs
   Then enter the password mate18 and tyoe the following commands:
     rosparam set cv_camera/device_id [Whatever number camera you want to access (0, 1, or 2)]
     rosrun cv_camera cv_camera_node
   Then minimize the terminal.
##########OLD#########################################################################

4) Open a new terminal and type the following command:
     ssh nagrrs@nagrrs
   Then enter the password "mate18" and type the following commands:
     roslaunch cameras slave_launch.launch
   Then minimize the terminal.

6) Open a new terminal and type the following command:
     rosrun rqt_image_view rqt_image_view image:=/cv_camera/image_raw
   Then minimize the terminal.

7) You may now open a new terminal to list and echo topics and debug.


To ssh to the udoo, do:
  ssh nagrrs@nagrrs

To upload arduino code:
  SSH into the udoo
  cd /home/nagrrs/Arduino/motorControl18
  arduino --upload motorControl18.ino --port /dev/ttyUSB0 --board arduino:avr:mega:cpu=atmega2560

FOR PANDA/CORDELL:
  To get two camera feeds: 
    Edit launch file to launch one cv_camera node
    Ssh in to create another cv_camera node
    Use rosrun rqt_image_view rqt_image_view
 to view (create 2 terminals to open twice)
