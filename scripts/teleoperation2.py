#!/usr/bin/env python3
# Import the required modules
import rospy
import sys
import termios
import tty
from geometry_msgs.msg import Twist

# Define a callback function for the keyboard input
def get_key():
    # Get the file descriptor of the terminal
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    # Change the terminal settings
    tty.setraw(sys.stdin.fileno())
    ch = sys.stdin.read(1)

    # Reset the terminal settings
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    return ch

# Define a callback function for the keyboard input
def keyboard_input():
    # Print the instructions for the user
    print("\nControl the turtle with the following keys:")
    print("a: Move one turtle up")
    print("b: Move down")
    print("c: Move left")
    print("d: Move right")
    print("e: Move clockwise")
    print("f: Move anti-clockwise")
    print("g: Stop\n")

    # Get the keyboard input
    key = get_key()
    
    # Set the velocity based on the key
    velocity_msg = Twist()
    if key == 'a':
        velocity_msg.linear.x = 1.0
        print("Moving up")
    elif key == 'b':
        velocity_msg.linear.x = -1.0
        print("Moving down")
    elif key == 'c':
        velocity_msg.angular.z = 1.0
        print("Moving left")
    elif key == 'd':
        velocity_msg.angular.z = -1.0
        print("Moving right")
    elif key == 'e':
        velocity_msg.angular.z = 1.0
        velocity_msg.linear.x = 1.0
        print("Moving clockwise")
    elif key == 'f':
        velocity_msg.angular.z = -1.0
        velocity_msg.linear.x = -1.0
        print("Moving anti-clockwise")
    elif key == 'g':
        velocity_msg.angular.z = 0
        velocity_msg.linear.x = 0
        print("Stopped")
    else:
        print("Invalid key")
        sys.exit()
    
    # Publish the velocity
    velocity_publisher.publish(velocity_msg)

# Initialize the node
rospy.init_node("teleoperation_node")

# Create a publisher for the velocity
velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

# Continuously get keyboard input
while not rospy.is_shutdown():
    keyboard_input()