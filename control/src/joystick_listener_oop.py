#! /usr/bin/env python

#  DESCRIPTION

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray, Bool, MultiArrayDimension
import numpy as np


class JoyControl:
    def hard_bound(self, value):
        if value < -1.0:
            value = -1.0
        elif value > 1.0:
            value = 1.0
        return value

    def scale_norm_vels_to_pwm(self, norm_vels, first, last):
        for i in range(first, last + 1):
            norm_vels[i] = (norm_vels[i] * 400.0 * 0.99) + 1500.0

        return norm_vels

    def scale_vels_by_max(self, vels, first, last):
        max_mag = abs(vels[first])

        for i in range(first + 1, last + 1):
            if abs(vels[i]) > max_mag:
                max_mag = abs(vels[i])

        if max_mag > 0.01:
            for i in range(first, last + 1):
                vels[i] = vels[i] / max_mag

        return vels

    def callback(self, data):
        buttons = data.buttons
        axes = data.axes

        # twist values assigned in the msg are in the range [-1.0, 1.0]
        self.twist_msg.linear.x = -self.hard_bound(axes[0])  # left stick left-right
        self.twist_msg.linear.y = self.hard_bound(axes[1])  # left stick up-down
        self.twist_msg.linear.z = ((-self.hard_bound(axes[4]) + 1.0) / 2.0 - (-self.hard_bound(axes[5]) + 1.0) / 2.0)
        self.twist_msg.angular.z = self.hard_bound(axes[2])  # right stick left-right, inverted so CCW is positive

        if buttons[6] and (not self.prev_light_button_state):  # Back button toggles light
            self.light_toggle = (not self.light_toggle)

        self.light_toggle_msg = self.light_toggle
        self.prev_light_button_state = buttons[6]

        if buttons[8] and (not self.e_stop):
            self.e_stop = True
        self.e_stop_msg = self.e_stop

        # translational velocity in the xy-plane
        self.vt = [self.twist_msg.linear.x, self.twist_msg.linear.y]
        # rotational velocity, with CCW rotation being positive
        self.w = self.twist_msg.angular.z

        self.front_left_vel = np.dot(self.vt, self.front_left_dir)
        self.front_right_vel = np.dot(self.vt, self.front_right_dir)
        self.back_left_vel = np.dot(self.vt, self.back_left_dir)
        self.back_right_vel = np.dot(self.vt, self.back_right_dir)

        if (abs(self.vt[0]) < 0.1) and (abs(self.vt[1]) < 0.1):
            self.front_left_vel = self.front_right_vel = self.back_left_vel = self.back_right_vel = 0.0

        if abs(self.w) > 0.1:
            self.front_left_vel -= self.w
            self.front_right_vel += self.w
            self.back_left_vel += self.w
            self.back_right_vel -= self.w

        self.front_up_vel = self.twist_msg.linear.z
        self.back_up_vel = -self.twist_msg.linear.z

        if (abs(axes[6]) < 0.1) and (abs(axes[7]) < 0.1):
            self.claw_grab = 0.0
        elif abs(axes[6]) > 0.1:
            self.claw_grab = (-axes[6] / abs(axes[6])) * 0.3
        elif abs(axes[7]) > 0.1:
            self.claw_grab = (axes[7] / abs(axes[7])) * 0.7

        #  claw rotation max speed is kept at 0 or 1/4 max speed
        #  B button rotates CW, X Button rotates CCW
        #  if rotation is being applied, the claw grab speed is applied in the opposite direction
        #    to cancel out the rotation's effect on it

        #  if X button is pressed, set rotate velocity negative (CCW) and grab velocity negative (out)
        if buttons[2]:
            self.claw_rotation = -0.25
            self.claw_grab = 0.5
        #  if B button is pressed, set rotate velocity positive (CW) and grab velocity positive (in)
        elif buttons[1]:
            self.claw_rotation = 0.25
            self.claw_grab = -0.5
        #  if neither are pressed, set speed zero
        else:
            self.claw_rotation = 0.0

        #  claw bottom max speed is kept at 1/4 max speed
        #  A button sets speed negative (backward)
        #  Y button sets speed positive (forward)
        if buttons[0]:
            self.claw_bottom = -1.0
        elif buttons[3]:
            self.claw_bottom = 1.0
        else:
            self.claw_bottom = 0.0

        self.motor_values = [self.front_left_vel, self.front_right_vel, self.back_left_vel, self.back_right_vel,
                             self.front_up_vel, self.back_up_vel, self.claw_grab, self.claw_rotation, self.claw_bottom]

        if self.e_stop:
            self.motor_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.light_toggle = False
            self.prev_light_button_state = False
            self.light_toggle_msg = False

        #  scale the first 4 motor values, corresponding to the xy-plane motors,
        #  using the max motor value as a norming factor
        #  motor_values = scaleValuesByMaxMotorValue(motor_values, 0, 3)

        #  scale the first 5 motor values, corresponding to the 4 xy-plane motors and the 2 z-axis motors,
        #  using the scaling factor and offset of the BlueRobotics Basic ESCs
        self.motor_values = self.scale_vels_by_max(self.motor_values, 0, 3)
        for i in range(0, 3):
            self.motor_values[i] = self.hard_bound(self.motor_values[i])

        self.motor_values = self.scale_norm_vels_to_pwm(self.motor_values, 0, 8)

        self.motor_values_msg.data = (self.motor_values[0], self.motor_values[1], self.motor_values[2],
                                      self.motor_values[3], self.motor_values[4], self.motor_values[5],
                                      self.motor_values[6], self.motor_values[7], self.motor_values[8])

        #  publish all messages to their respective topics
        self.twist_pub.publish(self.twist_msg)
        self.light_pub.publish(self.light_toggle_msg)
        self.motor_values_pub.publish(self.motor_values_msg)
        self.e_stop_pub.publish(self.e_stop_msg)

    def __init__(self):
        self.twist_pub = rospy.Publisher('transform', Twist, queue_size=5)
        self.light_pub = rospy.Publisher('light', Bool, queue_size=5)
        self.motor_values_pub = rospy.Publisher('motor_values', Int16MultiArray, queue_size=5)
        self.e_stop_pub = rospy.Publisher('e_stop', Bool, queue_size=5)

        self.prev_light_button_state = self.light_toggle = self.e_stop = False
        self.twist_msg = Twist()

        self.motor_values_msg = Int16MultiArray()
        self.motor_values_msg.layout.dim = [MultiArrayDimension('data', 1, 9)]

        self.light_toggle_msg = Bool()
        self.light_toggle_msg = False

        self.e_stop_msg = Bool()
        self.e_stop_msg = False

        #  direction vectors for each of the motors
        self.front_left_dir = [math.sqrt(2) / 2, math.sqrt(2) / 2]
        self.front_right_dir = [-math.sqrt(2) / 2, math.sqrt(2) / 2]
        self.back_left_dir = [math.sqrt(2) / 2, -math.sqrt(2) / 2]
        self.back_right_dir = [-math.sqrt(2) / 2, -math.sqrt(2) / 2]

        self.vt = [0.0, 0.0]
        self.w = 0.0

        self.front_left_vel = self.front_right_vel = self.back_left_vel = self.back_right_vel = 0.0

        self.front_up_vel = self.back_up_vel = 0.0

        self.claw_grab = self.claw_rotation = self.claw_bottom = 0.0
        self.motor_values = [0, 0, 0, 0, 0, 0, 0, 0, 0]

    def start_node(self):
        rospy.Subscriber('joy', Joy, self.callback)

        rospy.init_node('JoyListener')
        rospy.spin()


if __name__ == '__main__':
    try:
        joy_control = JoyControl()
        joy_control.start_node()
    except rospy.ROSInterruptException:
        pass
