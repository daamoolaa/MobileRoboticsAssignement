#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

def avoid_collision():
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('turtle_controller', anonymous=True)
    rate = rospy.Rate(10)

    # Get the current position of the turtle
    turtle_pose = Pose()
    def pose_callback(pose_message):
        turtle_pose.x = pose_message.x
        turtle_pose.y = pose_message.y
        turtle_pose.theta = pose_message.theta
    
    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)

    while not rospy.is_shutdown():
        # Check if the turtle is near the wall
        wall_collision = False
        if turtle_pose.x < 1 or turtle_pose.x > 9.5 or turtle_pose.y < 1 or turtle_pose.y > 9.5:
            wall_collision = True
        
        # Set the velocity
        velocity_message = Twist()
        if wall_collision:
            # Move away from the wall
            velocity_message.linear.x = -0.5
            velocity_message.angular.z = 0.5
        else:
            # Move forward
            velocity_message.linear.x = 0.5
            velocity_message.angular.z = 0.0
        
        pub.publish(velocity_message)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        avoid_collision()
    except rospy.ROSInterruptException:
        pass
