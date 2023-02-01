#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def avoid_collision():
    # Set up publisher to send velocity commands
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('avoid_collisions', anonymous=True)
    rate = rospy.Rate(10) # 10 Hz

    # Subscribe to turtle's pose to get its position
    turtle_pose = Pose()
    def pose_callback(pose_message):
        turtle_pose.x = pose_message.x
        turtle_pose.y = pose_message.y
    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)

    # Check for wall collisions and adjust velocity
    while not rospy.is_shutdown():
        velocity_msg = Twist()
        # Check if turtle is close to wall
        if turtle_pose.x < 1 or turtle_pose.x > 11 or turtle_pose.y < 1 or turtle_pose.y > 11:
            # Reverse direction
            velocity_msg.linear.x = -0.5
        else:
            # Move forward
            velocity_msg.linear.x = 0.5
        pub.publish(velocity_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        avoid_collision()
    except rospy.ROSInterruptException:
        pass
