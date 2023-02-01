#!/usr/bin/env python
import roslib
import tf
import rospy
import math
import random
from geometry_msgs.msg import Twist
import turtlesim.srv     
from turtlesim.msg import Pose

def vacuum_clean(turtle_id):
    # Set up publisher to send velocity commands
    pub = rospy.Publisher(f'/turtle{turtle_id}/cmd_vel', Twist, queue_size=10)
    rospy.init_node(f'vacuum_clean_{turtle_id}', anonymous=True)
    rate = rospy.Rate(10) # 10 Hz

    # Subscribe to turtle's pose to get its position
    turtle_pose = Pose()
    def pose_callback(pose_message):
        turtle_pose.x = pose_message.x
        turtle_pose.y = pose_message.y
        turtle_pose.theta = pose_message.theta
    rospy.Subscriber(f"/turtle{turtle_id}/pose", Pose, pose_callback)

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

        # Check if the turtle is close to a wall
        if turtle_pose.x < 1.0 or turtle_pose.x > 10.0 or turtle_pose.y < 1.0 or turtle_pose.y > 10.0:
            velocity_msg.linear.x = -5

        pub.publish(velocity_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        for turtle_id in [1, 2, 3]:
            rospy.wait_for_service('spawn')
            spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
            
            spawner(4*turtle_id, 2*turtle_id, 0, "turtle")
            rospy.wait_for_message(f"/turtle{turtle_id}/pose", Pose)
            vacuum_clean_thread = threading.Thread(target=vacuum_clean, args=(turtle_id,))
            vacuum_clean_thread.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
