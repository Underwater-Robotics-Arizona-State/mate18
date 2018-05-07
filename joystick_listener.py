#! /usr/bin/env python
# listens to the joy topic created by joy_node in the joy package of ROS
# and publishes the values converted into a Twist message as well as an array of motor values for the arduino to apply
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray, Bool, MultiArrayDimension

global twist_pub, light_pub, motor_values_pub, wifi_pub, prev_light_button_state, light_toggle
prev_light_button_state = False
light_toggle = False

def scaleNormValuesToPWM(norm_values, index_start, index_end):
    scaled_values = norm_values

    for i in range(index_start, index_end):
        scaled_values[i] = int((scaled_values[i] * 128.0) + 210.0)

    return scaled_values


def scaleValuesByMaxMotorValue(motor_values, index_start, index_end):
    down_scaled_values = motor_values

    max_value = math.fabs(motor_values[index_start])
    for i in range(index_start + 1, index_end):
        if math.fabs(motor_values[i]) > max:
            max_value = math.fabs(motor_values[i])

    if max_value != 0:
        for i in range(index_start, index_end):
            down_scaled_values[i] = motor_values[i] / (1.01 * max_value)

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


def callback(data):

    #  Xbox 360 Wired Controller for Linux Mapping
    #
    #  /joy.buttons
    #  Index | Button Name | Function
    #    0   |      A      | Claw Bottom Backward
    #    1   |      B      | Claw Rotation CW
    #    2   |      X      | Claw Rotation CCW
    #    3   |      Y      | Claw Bottom Forward
    #    4   |      LB     | Descend
    #    5   |      RB     | Ascend
    #    6   |     Back    | Toggle Light
    #    7   |     Start   | Enable wifi signal
    #    8   |     Power   | Unassigned
    #    9   |      LS     | Unassigned
    #   10   |      RS     | Unassigned
    #
    #  /joy.axes
    #  Index | Axis Name | Function
    #    0   |    LS X   | Direction x-component
    #    1   |    LS Y   | Direction y-component
    #    2   |     LT    | Open claw
    #    3   |    RS X   | Rotation
    #    4   |    RS Y   | Unassigned
    #    5   |     RT    | Close claw

    global twist_pub, light_pub, motor_values_pub, wifi_pub, prev_light_button_state, light_toggle
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
    twist_msg.linear.x = axes[0]  # left stick left-right
    twist_msg.linear.y = axes[1]  # left stick up-down
    twist_msg.angular.z = -axes[3]  # right stick left-right, inverted so CCW is positive

    if buttons[5]:  # RB = positive z = up
        twist_msg.linear.z = 1.0
    elif buttons[4]:  # LB = negative z = down
        twist_msg.linear.z = -1.0
    else:
        twist_msg.linear.z = 0.0

    if buttons[6]:  # Back button turns toggles light
        #  if the light is off and the button was just pressed, turn on the light
        if not light_toggle and not prev_light_button_state:
            light_toggle = True
            light_toggle_msg = True
        #  if the light is on and the button was just pressed, turn off the light
        elif light_toggle and not prev_light_button_state:
            light_toggle = False
            light_toggle_msg = False

        prev_light_button_state = buttons[6]

    if buttons[7]:  # Start button activates the wifi signal
        wifi_msg = True

    '''
    the labels refer to the index of each motor in the motor_values_msg.data array
    
    0       1
    /-------\
    |       |
    |       |
    |       |
    \-------/
    2       3
    
    '''

    vt = [twist_msg.linear.x, twist_msg.linear.y]  # translational velocity in x and y
    w = twist_msg.angular.z  # rotational velocity, with CCW rotation being positive

    v0 = [vt[0], -vt[1]]
    v1 = [-vt[0], -vt[1]]
    v2 = [vt[0], vt[1]]
    v3 = [-vt[0], vt[1]]

    v0_mag = vectorMagnitude(v0)
    v1_mag = vectorMagnitude(v1)
    v2_mag = vectorMagnitude(v2)
    v3_mag = vectorMagnitude(v3)

    if w > 0.05:  # if rotating CCW
        v0_mag += w
        v3_mag += w
    elif w < 0.05:  # if rotating CW
        v1_mag -= w
        v2_mag -= w

    #  v4 is the velocity of the front z-axis motor oriented down
    #  v5 is the velocity of the back z-axis motor oriented down
    v4_mag = twist_msg.linear.z
    v5_mag = twist_msg.linear.z

    #  Claw grab velocity is set to the magnitude of RT minus the magnitude of LT, both of which range from 0.0 to 1.0
    claw_grab = axes[5] - axes[2]

    #  claw rotation magnitude is kept at 0 or 1/4 max speed
    #  B button rotates CW, X Button rotates CCW
    claw_rotation = 0.0
    #  if X button is pressed, set speed negative (CCW)
    if buttons[2]:
        claw_rotation = -0.25
    #  if B button is pressed, set speed positive (CW)
    elif buttons[1]:
        claw_rotation = 0.25
    #  if neither are pressed, set speed zero
    else:
        claw_rotation = 0.0

    #  claw bottom magnitude is kept at 0 or 1/4 max speed
    #  A button sets speed negative (backward)
    #  Y button sets speed positive (forward)
    claw_bottom = 0.0
    if buttons[0]:
        claw_bottom = -0.25
    elif buttons[3]:
        claw_bottom = 0.25
    else:
        claw_bottom = 0.0

    motor_values = [v0_mag, v1_mag, v2_mag, v3_mag, v4_mag, v5_mag, claw_grab, claw_rotation, claw_bottom]

    #  scale the first 4 motor values, corresponding to the xy-plane motors,
    #  using the max motor value as a norming factor
    motor_values = scaleValuesByMaxMotorValue(motor_values, 0, 3)

    #  scale the first 5 motor values, corresponding to the 4 xy-plane motors and the 2 z-axis motors,
    #  using the scaling factor and offset of the BlueRobotics Basic ESCs
    motor_values_pwm = scaleNormValuesToPWM(motor_values, 0, 8)

    motor_values_msg.data = (motor_values_pwm[0], motor_values_pwm[1], motor_values_pwm[2], motor_values_pwm[3],
                             motor_values_pwm[4], motor_values_pwm[5], motor_values_pwm[6], motor_values_pwm[7],
                             motor_values_pwm[8])

    #  publish all messages to their respective topics
    twist_pub.publish(twist_msg)
    light_pub.publish(light_toggle_msg)
    motor_values_pub.publish(motor_values_msg)
    wifi_pub.publish(wifi_msg)


def init():
    global twist_pub, light_pub, motor_values_pub, wifi_pub, prev_light_button_state, light_toggle

    twist_pub = rospy.Publisher('twistTransform', Twist)
    light_pub = rospy.Publisher('light', Bool)
    motor_values_pub = rospy.Publisher('motorValues', Int16MultiArray)
    wifi_pub = rospy.Publisher('wifi', Bool)

    rospy.Subscriber("joy", Joy, callback)

    rospy.init_node('JoyListener')
    rospy.spin()


if __name__ == '__main__':
    init()
