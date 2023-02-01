#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# Callback function for subscriber to turtle pose
def pose_callback(pose_msg, turtle_number):
    global x_goal, y_goal
    turtle_name = "turtle" + str(turtle_number)

    # Get turtle position
    x_pos = pose_msg.x
    y_pos = pose_msg.y

    # Calculate Euclidean distance to goal
    distance = math.sqrt((x_goal - x_pos) ** 2 + (y_goal - y_pos) ** 2)

    # Set linear velocity proportional to distance
    linear_vel = 0.5 * distance

    # Set angular velocity to steer the turtle towards the goal
    angle = math.atan2(y_goal - y_pos, x_goal - x_pos) - pose_msg.theta
    angular_vel = 4 * angle

    # Publish velocity commands to the turtle
    velocity_cmd = Twist()
    velocity_cmd.linear.x = linear_vel
    velocity_cmd.angular.z = angular_vel
    velocity_publisher = rospy.Publisher(turtle_name + '/cmd_vel', Twist, queue_size=10)
    velocity_publisher.publish(velocity_cmd)

if __name__ == '__main__':
    global x_goal, y_goal

    # Initialize ROS node
    rospy.init_node('vacuum_cleaning_multiple_turtles')

    # Number of turtles to use for cleaning
    num_turtles = 3

    # Set goal position for all turtles
    x_goal = 5.5
    y_goal = 5.5

    # Subscribe to each turtle's pose topic
    for i in range(num_turtles):
        turtle_name = "turtle" + str(i + 1)
        pose_subscriber = rospy.Subscriber(turtle_name + '/pose', Pose, pose_callback, (i + 1))

    # Spin until node is shutdown
    rospy.spin()
