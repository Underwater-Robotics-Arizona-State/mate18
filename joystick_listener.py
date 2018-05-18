#! /usr/bin/env python
# listens to the joy topic created by joy_node in the joy package of ROS
# and publishes the values converted into a Twist message as well as an array of motor values for the arduino to apply
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray, Bool, MultiArrayDimension
import numpy as np

global twist_pub, light_pub, motor_values_pub, wifi_pub, prev_light_button_state, light_toggle, e_stop
prev_light_button_state = False
light_toggle = False
e_stop = False


def scaleNormValuesToPWM(norm_values, index_start, index_end):
    scaled_values = norm_values

    for i in range(index_start, index_end + 1):
        scaled_values[i] = int((scaled_values[i] * 400) + 1500)

    return scaled_values


def scaleValuesByMaxMotorValue(motor_values, index_start, index_end):
    down_scaled_values = motor_values

    max_value = math.fabs(motor_values[index_start])
    for i in range(index_start + 1, index_end + 1):
        if math.fabs(motor_values[i]) > max:
            max_value = math.fabs(motor_values[i])

    if math.fabs(max_value) > 0:
        for i in range(index_start, index_end + 1):
            down_scaled_values[i] = motor_values[i] / (max_value)

    return down_scaled_values


def vectorMagnitude(vector):
    return math.sqrt(vector[0] * vector[0] + vector[1] * vector[1])


def normVector(vector):
    normed_vector = vector

    magnitude = 0.99 * vectorMagnitude(vector)
    if magnitude != 0:
        normed_vector[0] = normed_vector[0] / magnitude
        normed_vector[1] = normed_vector[1] / magnitude

    return normed_vector


def norm_bound(value):
    if value > 1.0:
        return 1.0
    elif value < -1.0:
        return -1.0
    else:
        return value


def callback(data):

    #  Xbox 360 Wired Controller for Linux Mapping
    #
    #  /joy.buttons
    #  Index | Button Name | Function
    #    0   |      A      | Claw Bottom Backward
    #    1   |      B      | Claw Rotation CW
    #    2   |      X      | Claw Rotation CCW
    #    3   |      Y      | Claw Bottom Forward
    #    4   |      LB     | Unassigned
    #    5   |      RB     | Unassigned
    #    6   |     Back    | Toggle Light
    #    7   |     Start   | Enable wifi signal
    #    8   |     Power   | Emergency Stop
    #    9   |      LS     | Unassigned
    #   10   |      RS     | Unassigned
    #
    #  /joy.axes
    #  Index | Axis Name | Function
    #    0   |    LS X   | Direction x-component
    #    1   |    LS Y   | Direction y-component
    #    2   |     LT    | Descend
    #    3   |    RS X   | Unassigned
    #    4   |    RS Y   | Rotate yaw
    #    5   |     RT    | Ascend
    #    6   |  D-pad X  | 1/3 speed open/close claw
    #    7   |  D-pad Y  | 2/3 speed open/close claw

    global twist_pub, light_pub, motor_values_pub, wifi_pub, prev_light_button_state, light_toggle, e_stop
    twist_msg = Twist()
    motor_values_msg = Int16MultiArray()
    motor_values_msg.layout.dim = [MultiArrayDimension('data', 1, 9)]
    light_toggle_msg = Bool()
    wifi_msg = Bool()
    wifi_msg = False
    motor_values = []

    buttons = data.buttons
    axes = data.axes

    # twist values assigned in the msg are in the range [-1.0, 1.0]
    twist_msg.linear.x = -norm_bound(axes[0])  # left stick left-right
    twist_msg.linear.y = norm_bound(axes[1])  # left stick up-down
    twist_msg.angular.z = norm_bound(axes[2])  # right stick left-right, inverted so CCW is positive

    twist_msg.linear.z = ((-norm_bound(axes[4]) + 1.0) / 2.0 - (-norm_bound(axes[5]) + 1.0) / 2.0)

    if buttons[6] and (not prev_light_button_state):  # Back button toggles light
        light_toggle = (not light_toggle)

    light_toggle_msg = light_toggle
    prev_light_button_state = buttons[6]

    if buttons[7]:  # Start button activates the wifi signal
        wifi_msg = True

    if buttons[8] and (not e_stop):
        e_stop = True

    '''
    the labels refer to the index of each motor in the motor_values_msg.data array
    
     /      \
    0-------1
   /|   4   |\
    |       |
   \|   5   |/
    2-------3
     \     /  
    
    '''
    
    vt = [twist_msg.linear.x, twist_msg.linear.y]

    w = twist_msg.angular.z  # rotational velocity, with CCW rotation being positive

    #  direction vectors for each of the motors
    #u0 = [math.sqrt(2) / 2, math.sqrt(2) / 2]
    #u1 = [-math.sqrt(2) / 2, math.sqrt(2) / 2]
    #u2 = [math.sqrt(2) / 2, -math.sqrt(2) / 2]
    #u3 = [-math.sqrt(2) / 2, -math.sqrt(2) / 2]
    

    #v0_mag = np.dot(vt, u0)
    #v1_mag = np.dot(vt, u1)
    #v2_mag = np.dot(vt, u2)
    #v3_mag = np.dot(vt, u3)

    v0_mag = 0.0
    v1_mag = 0.0
    v2_mag = 0.0
    v3_mag = 0.0

    if (math.fabs(vt[0]) < 0.1) and (math.fabs(vt[1]) < 0.1):
        v0_mag = v1_mag = v2_mag = v3_mag = 0.0
    elif math.fabs(vt[0]) > math.fabs(vt[1]):
        v0_mag = vt[0]
        v1_mag = vt[0]
        v2_mag = -vt[0] 
        v3_mag = -vt[0]
    else:
        v0_mag = vt[1]
        v1_mag = -vt[1]
        v2_mag = vt[1]
        v3_mag = -vt[1]

    if math.fabs(w) > 0.1:
        v0_mag = -w
        v1_mag = w
        v2_mag = w
        v3_mag = -w


    #  v4 is the velocity of the front z-axis motor oriented down
    #  v5 is the velocity of the back z-axis motor oriented down
    v4_mag = twist_msg.linear.z
    v5_mag = -twist_msg.linear.z

    claw_grab = 0.0
    if (math.fabs(axes[6]) < 0.1) and (math.fabs(axes[7]) < 0.1):
        claw_grab = 0.0
    elif (math.fabs(axes[6]) > 0.1):
        claw_grab = (-axes[6] / math.fabs(axes[6])) * 0.3
    elif (math.fabs(axes[7]) > 0.1):
        claw_grab = (axes[7] / math.fabs(axes[7])) * 0.7
    

    #  claw rotation max speed is kept at 0 or 1/4 max speed
    #  B button rotates CW, X Button rotates CCW
    #  if rotation is being applied, the claw grab speed is applied in the opposite direction to cancel out the rotation's effect on it
    claw_rotation = 0.0
    #  if X button is pressed, set rotate velocity negative (CCW) and grab velocity negative (out)
    if buttons[2]:
        claw_rotation = -0.25
        claw_grab = 0.5
    #  if B button is pressed, set rotate velocity positive (CW) and grab velocity positive (in)
    elif buttons[1]:
        claw_rotation = 0.25
        claw_grab = -0.5
    #  if neither are pressed, set speed zero
    else:
        claw_rotation = 0.0

    #  claw bottom max speed is kept at 1/4 max speed
    #  A button sets speed negative (backward)
    #  Y button sets speed positive (forward)
    claw_bottom = 0.0
    if buttons[0]:
        claw_bottom = -1
    elif buttons[3]:
        claw_bottom = 1
    else:
        claw_bottom = 0.0

    motor_values = [v0_mag, v1_mag, v2_mag, v3_mag, v4_mag, v5_mag, claw_grab, claw_rotation, claw_bottom]

    if e_stop:
        motor_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        light_toggle = False
        prev_light_button_state = False
        light_toggle_msg = False
        wifi_msg = False

    #  scale the first 4 motor values, corresponding to the xy-plane motors,
    #  using the max motor value as a norming factor
    #motor_values = scaleValuesByMaxMotorValue(motor_values, 0, 3)

    #  scale the first 5 motor values, corresponding to the 4 xy-plane motors and the 2 z-axis motors,
    #  using the scaling factor and offset of the BlueRobotics Basic ESCs
    motor_values = scaleNormValuesToPWM(motor_values, 0, 8)

    motor_values_msg.data = (motor_values[0], motor_values[1], motor_values[2], motor_values[3],
                             motor_values[4], motor_values[5], motor_values[6], motor_values[7],
                             motor_values[8])

    #  publish all messages to their respective topics
    twist_pub.publish(twist_msg)
    light_pub.publish(light_toggle_msg)
    motor_values_pub.publish(motor_values_msg)
    wifi_pub.publish(wifi_msg)


def init():
    global twist_pub, light_pub, motor_values_pub, wifi_pub, prev_light_button_state, light_toggle, e_stop

    twist_pub = rospy.Publisher('transform', Twist, queue_size=1)
    light_pub = rospy.Publisher('light', Bool, queue_size=1)
    motor_values_pub = rospy.Publisher('motorValues', Int16MultiArray, queue_size = 1)
    wifi_pub = rospy.Publisher('wifi', Bool, queue_size=1)

    rospy.Subscriber("joy", Joy, callback)

    rospy.init_node('JoyListener')
    rospy.spin()


if __name__ == '__main__':
    init()
