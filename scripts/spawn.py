#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn

rospy.init_node('spawn_turtle')
rospy.wait_for_service('spawn')
	
spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
spawn_turtle(2.0, 3, 1.7, "turtle2")
# spawn_turtle(4, 4, 1.7, "turtle3")
