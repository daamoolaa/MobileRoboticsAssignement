#!/usr/bin/env python
import rospy
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
        turtle_pose.theta = pose_message.theta
    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)

    # Set the initial direction of the turtle
    direction = 1

    # Control loop to move turtle in a snake pattern
    while not rospy.is_shutdown():
        velocity_msg = Twist()

        # Check if the turtle is near the edge of the window
        if turtle_pose.x < 1 or turtle_pose.x > 10 or turtle_pose.y < 1 or turtle_pose.y > 10:
            # Change direction
            direction *= -1

        # Move the turtle in the chosen direction
        velocity_msg.linear.x = 0.5
        velocity_msg.angular.z = direction * 5

        pub.publish(velocity_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        vacuum_clean()
    except rospy.ROSInterruptException:
        pass
