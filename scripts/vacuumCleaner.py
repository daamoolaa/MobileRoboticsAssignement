#!/usr/bin/env python
import imghdr
import rospy
import math
import random
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def vacuum_clean():
    # Set up publisher to send velocity commands
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('vacuum_clean', anonymous=True)
    rate = rospy.Rate(10) # 10 Hz

    # Subscribe to turtle's pose to get its position
    turtle_pose = Pose()
    def pose_callback(pose_message):
        turtle_pose.x = pose_message.x
        turtle_pose.y = pose_message.y
    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)

    # Set the goal position for the turtle
    goal_x = 11
    goal_y = 11

    # Control loop to move turtle towards goal
    while not rospy.is_shutdown():
        velocity_msg = Twist()

        # Calculate the angle between the turtle and the goal
        angle = math.atan2(goal_y - turtle_pose.y, goal_x - turtle_pose.x)

        # Check if the turtle has reached the goal
        if abs(turtle_pose.x - goal_x) < 0.1 and abs(turtle_pose.y - goal_y) < 0.1:
            # Choose a new goal
            goal_x = 11 * random.random()
            goal_y = 11 * random.random()
        else:
            # Move towards the goal
            velocity_msg.linear.x = 5
            velocity_msg.angular.z = 20 * (angle - turtle_pose.theta)
             # Check if turtle1 is about to hit a wall and override movement
            if turtle_pose.x < 0.5 or turtle_pose.x > 11.5 or turtle_pose.y < 0.5 or turtle_pose.y > 11.5:
                velocity_msg.linear.x = -1
                velocity_msg.angular.z = 20

        pub.publish(velocity_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        vacuum_clean()
    except rospy.ROSInterruptException:
        pass
