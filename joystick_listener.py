#! /usr/bin/env python
# listens to the joy topic created by joy_node in the joy package of ROS
# and publishes the values converted into a Twist message
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray, Bool, MultiArrayDimension


def scaleNormValuesToPWM(norm_values, index_start, index_end):
    scaled_values = norm_values

    for i in range(index_start, index_end):
        scaled_values[i] = int((scaled_values[i] * 0.99 * 256) + 82)

    return scaled_values


def scaleValuesByMaxMotorValue(motor_values, index_start, index_end):
    down_scaled_values = motor_values

    max = math.fabs(motor_values[index_start])
    for i in range(index_start + 1, index_end):
        if math.fabs(motor_values[i]) > max:
            max = math.fabs(motor_values[i])

    if max != 0:
        for i in range(index_start, index_end):
            down_scaled_values[i] = motor_values[i] / (1.01 * max)

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
    twist_msg = Twist()
    motor_values_msg = Int16MultiArray()
    motor_values_msg.layout.dim = [MultiArrayDimension('data', 1, 9)]
    light_toggle_msg = Bool()
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


    if buttons[3]:  # Y button turns light on
        light_toggle_msg = True
    elif buttons[0]:  # A button turns light off
        light_toggle_msg = False

    '''
    the labels refer to the index of each motor in the motor_values_msg.data array
    
    1       0
    /-------\
    |       |
    |       |
    |       |
    \-------/
    3       2
    
    '''

    vt = [twist_msg.linear.x, twist_msg.linear.y]  # translational velocity in x and y
    w = twist_msg.angular.z  # rotational velocity, with CCW rotation being positive

    v0 = [vt[0], vt[1]]
    v1 = [-vt[0], vt[1]]
    v2 = [vt[0], -vt[1]]
    v3 = [-vt[0], -vt[1]]

    v0_mag = vectorMagnitude(v0)
    v1_mag = vectorMagnitude(v1)
    v2_mag = vectorMagnitude(v2)
    v3_mag = vectorMagnitude(v3)

    if w > 0.05:  # if rotating CCW
        v1_mag -= w
        v2_mag += w
    elif w < 0.05:  # if rotating CW
        v1_mag += w
        v2_mag -= w

    #  v4 is the velocity of the front z-axis motor oriented down
    #  v5 is the velocity of the back z-axis motor oriented down
    v4_mag = 0.0
    v5_mag = 0.0
    if twist_msg.linear.z > 0:
        v4_mag = twist_msg.linear.z
        v5_mag = twist_msg.linear.z
    elif twist_msg.linear.z < 0:
        v4_mag = -twist_msg.linear.z
        v5_mag = -twist_msg.linear.z
    else:
        v4_mag = 0.0
        v5_mag = 0.0

    claw_grab = 0.0
    claw_rotation = 0.0
    claw_bottom = 0.0

    motor_values = [v0_mag, v1_mag, v2_mag, v3_mag, v4_mag, v5_mag, claw_grab, claw_rotation, claw_bottom]


    #  scale the first 4 motor values, corresponding to the xy-plane motors,
    #  using the max motor value as a norming factor
    motor_values = scaleValuesByMaxMotorValue(motor_values, 0, 3)

    #  scale the first 5 motor values, corresponding to the 4 xy-plane motors and the 2 z-axis motors,
    #  using the scaling factor and offset of the BlueRobotics Basic ESCs
    motor_values = scaleNormValuesToPWM(motor_values, 0, 8)

    motor_values_msg.data = (motor_values[0], motor_values[1], motor_values[2], motor_values[3], motor_values[4], motor_values[5], motor_values[6], motor_values[7], motor_values[8])

    #  publish all messages to their respective topics
    twist_pub.publish(twist_msg)
    light_pub.publish(light_toggle_msg)
    motor_values_pub.publish(motor_values_msg)




def init():
    global twist_pub
    global light_pub
    global motor_values_pub
    twist_pub = rospy.Publisher('twistTransform', Twist)
    light_pub = rospy.Publisher('light', Bool)
    motor_values_pub = rospy.Publisher('motorValues', Int16MultiArray)

    rospy.Subscriber("joy", Joy, callback)

    rospy.init_node('JoyListener')
    rospy.spin()


if __name__ == '__main__':
    init()
