#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from random import uniform
from time import sleep
import random
from turtlesim.srv import SetPen

class Turtle:
    def __init__(self, color):
        self.color = color
        self.pubOne = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pubTwo = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

        penOne = rospy.ServiceProxy('/turtle1/set_pen',SetPen)
        penTwo = rospy.ServiceProxy('/turtle2/set_pen', SetPen)

        penOne(0, 0,0,30, 0)
        penTwo(0, 0,0,30,0)
        self.twistOner = Twist()
        
        self.twistOner.linear.x = -random.uniform(1, 3)
        self.twistOner.angular.z = -random.uniform(-1, 3)

        self.twistTwoer = Twist()
        
        self.twistTwoer.linear.x = -random.uniform(1, 3)
        self.twistTwoer.angular.z = random.uniform(-1, 3)
    def moveOne(self):
        twistOne = Twist()
        print("turn one")
        twistOne.linear.x = random.uniform(1, 5)
        twistOne.angular.z = -random.uniform(1, 5) 
        self.pubOne.publish(twistOne)
  
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