#!/usr/bin/env python
import rospy
import math
import random
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def vacuum_clean(turtle_num):
    # Set up publisher to send velocity commands
    pub = rospy.Publisher('/turtle' + str(turtle_num) + '/cmd_vel', Twist, queue_size=10)
    rospy.init_node('vacuum_clean' + str(turtle_num), anonymous=True)
    rate = rospy.Rate(10) # 10 Hz

    # Subscribe to turtle's pose to get its position
    turtle_pose = Pose()
    def pose_callback(pose_message):
        turtle_pose.x = pose_message.x
        turtle_pose.y = pose_message.y
        turtle_pose.theta = pose_message.theta
    rospy.Subscriber("/turtle" + str(turtle_num) + "/pose", Pose, pose_callback)

    # Set the goal position for the turtle
    goal_x = 11 * random.random()
    goal_y = 11 * random.random()

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

        pub.publish(velocity_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        for i in range(3):
            rospy.loginfo("Starting turtle vacuum cleaning behavior %d" % (i+1))
            rospy.init_node("vacuum_clean_turtle_" + str(i+1), anonymous=True)
            vacuum_clean(i+1)
    except rospy.ROSInterruptException:
        pass
