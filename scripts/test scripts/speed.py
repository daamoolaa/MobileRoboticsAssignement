#!/usr/bin/env python
import rospy
import sys, select, termios, tty
from geometry_msgs.msg import Twist

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    # Initialize node and publisher
    rospy.init_node('teleop_turtle')
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    # Save the terminal settings
    settings = termios.tcgetattr(sys.stdin)

    # Set the speed of the turtle
    speed = 1.0

    while not rospy.is_shutdown():
        # Create Twist message
        vel_msg = Twist()
        vel_msg.linear.x = speed

        # Read the keyboard input
        key = getKey()

        # Increase the speed if the "+" key is pressed
        if key == '+':
            speed += 0.1
        # Decrease the speed if the "-" key is pressed
        elif key == '-':
            speed -= 0.1

        # Limit the speed between 0 and 2
        if speed > 2.0:
            speed = 2.0
        elif speed < 0.0:
            speed = 0.0

        # Publish the velocity message
        pub.publish(vel_msg)
