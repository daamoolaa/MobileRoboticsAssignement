#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from random import uniform
from time import sleep
import random
from turtlesim.srv import SetPen
#define the turtle class
class Turtle:
    def __init__(self, color):
        #initialize the color attribute and create two publishers for the two turtles
        self.color = color
        self.pubOne = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pubTwo = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
        #create two service proxies to set pen details for the two turtles
        penOne = rospy.ServiceProxy('/turtle1/set_pen',SetPen)
        penTwo = rospy.ServiceProxy('/turtle2/set_pen', SetPen)
        #set size for the pen
        penOne(0, 0,0,30, 0)
        penTwo(0, 0,0,30,0)
        # Create two publishers for the two turtles
        self.twistOner = Twist()
        # Set the linear velocity and angular velocity for turtle 1 randomly in the range
        self.twistOner.linear.x = -random.uniform(1, 3)
        self.twistOner.angular.z = -random.uniform(-1, 3)

        self.twistTwoer = Twist()
        # Set the linear velocity and angular velocity for turtle 1 randomly in the range
        self.twistTwoer.linear.x = -random.uniform(1, 3)
        self.twistTwoer.angular.z = random.uniform(-1, 3)
    #method to move turtle 1
    def moveOne(self):
        twistOne = Twist()
        print("turn one")
        twistOne.linear.x = random.uniform(1, 5)
        twistOne.angular.z = -random.uniform(1, 5) 
        self.pubOne.publish(twistOne)
    #method to move turtle 2
    def moveTwo(self):

        twistTwo = Twist()
        print("turn two")
        twistTwo.linear.x = -random.uniform(1, 5)
        twistTwo.angular.z = random.uniform(1, 5) 
        self.pubTwo.publish(twistTwo)
# def check_collision(turtle1, turtle2):
#     if abs(turtle1.twist.linear.x - turtle2.twist.linear.x) < 0.1 and abs(turtle1.twist.angular.z - turtle2.twist.angular.z) < 0.1:
#         print("hey")
#         turtle2.twist.linear.x = random.uniform(1, 5)
#         turtle2.twist.angular.z = -random.uniform(1, 5) 

if __name__ == '__main__':
    rospy.init_node('turtle_random_movement')
    turtle1 = Turtle('blue')

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        turtle1.moveOne()
        turtle1.moveTwo()
        rate.sleep()


