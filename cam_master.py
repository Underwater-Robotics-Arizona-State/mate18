#!/usr/bin/env python
#reads camera selection and publishes frames

import numpy as np
import roslib
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

def cam_master():
    # opencv camera capture variables
    front_cam_left_cap = cv2.VideoCapture(0)
    front_cam_right_cap = cv2.VideoCapture(1)
    bottom_cam_cap = cv2.VideoCapture(2)
    bridge = CvBridge()

    # ros node variables and initiation
    front_cam_left_pub = rospy.Publisher("front_cam/left", Image, queue_size=2)
    front_cam_right_pub = rospy.Publisher("front_cam/right", Image, queue_size=2)
    bottom_cam_pub = rospy.Publisher("bottom_cam", Image, queue_size=2)
    rospy.init_node('cam_master', anonymous=True)
    rate = rospy.Rate(30)  # 30hz (30fps)

    while not rospy.is_shutdown():
        front_cam_left_frame = front_cam_left_cap.read()
        front_cam_right_frame = front_cam_right_cap.read()
        bottom_cam_frame = bottom_cam_cap.read()

        # image scaling or modification should happen here if necessary

        # converting the frames to ros images
        front_cam_left_frame_msg = bridge.cv2_to_imgmsg(front_cam_left_frame, encoding="passthrough")
        front_cam_right_frame_msg = bridge.cv2_to_imgmsg(front_cam_right_frame, encoding="passthrough")
        bottom_cam_frame_msg = bridge.cv2_to_imgmsg(bottom_cam_frame, encoding="passthrough")

        rospy.loginfo(front_cam_left_frame_msg)
        rospy.loginfo(front_cam_right_frame_msg)
        rospy.loginfo(bottom_cam_frame_msg)

        front_cam_left_pub.publish(front_cam_left_frame_msg)
        front_cam_right_pub.publish(front_cam_right_frame_msg)
        bottom_cam_pub.publish(bottom_cam_frame_msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        cam_master()
    except rospy.ROSInterruptException:
        pass
