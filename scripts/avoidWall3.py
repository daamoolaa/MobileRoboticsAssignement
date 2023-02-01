#!/usr/bin/env python
import random
import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def avoid_wall_collision():
    # Set up publisher to send velocity commands
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('avoid_wall_collision', anonymous=True)
    rate = rospy.Rate(10) # 10 Hz

    # Subscribe to turtle's pose to get its position
    turtle_pose = Pose()
    def pose_callback(pose_message):
        turtle_pose.x = pose_message.x
        turtle_pose.y = pose_message.y
        turtle_pose.theta = pose_message.theta
    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)

    # Set the goal position for the turtle
    goal_x = 10
    goal_y = 10

    # Control loop to move turtle towards goal while avoiding walls
    while not rospy.is_shutdown():
        velocity_msg = Twist()

        # Calculate the angle between the turtle and the goal
        angle = math.atan2(goal_y - turtle_pose.y, goal_x - turtle_pose.x)

        # Check if the turtle is about to hit a wall
        if turtle_pose.x >= 10 or turtle_pose.x <= 0 or turtle_pose.y >= 10 or turtle_pose.y <= 0:
            # Override movement and move away from the wall
            velocity_msg.linear.x = -0.5
            velocity_msg.angular.z = 20 * (angle + math.pi/2 - turtle_pose.theta)
        else:
            # Check if the turtle has reached the goal
            if abs(turtle_pose.x - goal_x) < 0.1 and abs(turtle_pose.y - goal_y) < 0.1:
                # Choose a new goal
                goal_x = 10 * random.random()
                goal_y = 10 * random.random()
            else:
                # Move towards the goal
                velocity_msg.linear.x = 5
                velocity_msg.angular.z = 20 * (angle - turtle_pose.theta)

        pub.publish(velocity_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        avoid_wall_collision()
    except rospy.ROSInterruptException:
        pass
