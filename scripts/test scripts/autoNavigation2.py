#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

def move_to_goal(x_goal, y_goal):
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
        # Calculate the linear and angular velocities
        linear_velocity = 0.5 * ((x_goal - turtle_pose.x) ** 2 + (y_goal - turtle_pose.y) ** 2) ** 0.5
        angular_velocity = 4 * (math.atan2(y_goal - turtle_pose.y, x_goal - turtle_pose.x) - turtle_pose.theta)
        
        # Set the velocity
        velocity_message = Twist()
        velocity_message.linear.x = linear_velocity
        velocity_message.angular.z = angular_velocity
        pub.publish(velocity_message)
        
        if abs(turtle_pose.x - x_goal) < 0.1 and abs(turtle_pose.y - y_goal) < 0.1:
            break
        
        rate.sleep()

if __name__ == '__main__':
    try:
        move_to_goal(5, 5)
    except rospy.ROSInterruptException:
        pass
