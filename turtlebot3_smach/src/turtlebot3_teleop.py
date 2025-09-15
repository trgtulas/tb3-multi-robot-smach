#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty
import os

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84
WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82
LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop

CTRL-C to quit
"""

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def constrain(input_vel, low_bound, high_bound):
    return min(max(input_vel, low_bound), high_bound)

def check_linear_limit_velocity(vel):
    if TURTLEBOT3_MODEL == 'burger':
        return constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    else:
        return constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)

def check_angular_limit_velocity(vel):
    if TURTLEBOT3_MODEL == 'burger':
        return constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    else:
        return constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)

def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    return output

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_keyboard')
    pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)

    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0

    try:
        print(msg)
        while not rospy.is_shutdown():
            key = get_key()
            if key == 'w':
                target_linear_velocity = check_linear_limit_velocity(target_linear_velocity + LIN_VEL_STEP_SIZE)
            elif key == 'x':
                target_linear_velocity = check_linear_limit_velocity(target_linear_velocity - LIN_VEL_STEP_SIZE)
            elif key == 'a':
                target_angular_velocity = check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
            elif key == 'd':
                target_angular_velocity = check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
            elif key == ' ' or key == 's':
                target_linear_velocity = 0.0
                control_linear_velocity = 0.0
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0
            elif key == '\x03':
                break

            control_linear_velocity = make_simple_profile(control_linear_velocity, target_linear_velocity, (LIN_VEL_STEP_SIZE/2.0))
            control_angular_velocity = make_simple_profile(control_angular_velocity, target_angular_velocity, (ANG_VEL_STEP_SIZE/2.0))

            twist = Twist()
            twist.linear.x = control_linear_velocity
            twist.angular.z = control_angular_velocity

            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
