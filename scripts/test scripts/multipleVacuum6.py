import rospy
import random
import math
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, TeleportAbsolute
from turtlesim.msg import Pose

def spawn_turtle(x, y, theta):
    # Wait for the spawn service to become available
    rospy.wait_for_service('spawn')
    try:
        # Create a service proxy to call the spawn service
        spawner = rospy.ServiceProxy('spawn', Spawn)
        # Call the spawn service and get the name of the new turtle
        turtle_name = spawner(x, y, theta).name
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
    return turtle_name

def vacuum_clean(turtle_name):
    # Set up publisher to send velocity commands
    pub = rospy.Publisher(turtle_name + '/cmd_vel', Twist, queue_size=10)
    rospy.init_node('vacuum_clean', anonymous=True)
    rate = rospy.Rate(10) # 10 Hz

    # Subscribe to turtle's pose to get its position
    turtle_pose = Pose()
    def pose_callback(pose_message):
        turtle_pose.x = pose_message.x
        turtle_pose.y = pose_message.y
        turtle_pose.theta = pose_message.theta
    rospy.Subscriber(turtle_name + '/pose', Pose, pose_callback)

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

        pub.publish(velocity_msg)
        rate.sleep()

if __name__ == '__main__':
    # Spawn three turtles at different locations
    turtle1_name = spawn_turtle(1, 1, 0)
    turtle2_name = spawn_turtle(5, 5, 0)
    turtle3_name = spawn_turtle(9, 9, 0)

    # Start vacuum cleaning behavior for each turtle in a separate thread
    rospy.Timer(rospy.Duration(0.1), lambda: vacuum_clean(turtle1_name))
    rospy.Timer(rospy.Duration(0.2), lambda: vacuum_clean(turtle2_name))
    rospy.Timer(rospy.Duration(0.2), lambda: vacuum_clean(turtle3_name))
