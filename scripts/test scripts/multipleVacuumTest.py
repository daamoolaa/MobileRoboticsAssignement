#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from random import uniform
from time import sleep
import math 
import turtlesim.msg

class Turtle:
    def __init__(self, name, color):
        self.name = name
        self.color = color
        self.pub = rospy.Publisher(self.name + '/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.twist.linear.x = uniform(0.1, 0.5)
        self.twist.angular.z = uniform(-0.5, 0.5)
    
    def move(self):
        self.pub.publish(self.twist)

def check_collision(turtle1, turtle2):
    if abs(turtle1.twist.linear.x - turtle2.twist.linear.x) < 0.1 and abs(turtle1.twist.angular.z - turtle2.twist.angular.z) < 0.1:
        turtle2.twist.linear.x = uniform(0.1, 0.5)
        turtle2.twist.angular.z = uniform(-0.5, 0.5)

if __name__ == '__main__':
    rospy.init_node('turtle_random_movement')
    turtle1 = Turtle('turtle1', 'blue')
    turtle2 = Turtle('turtle2', 'red')
    turtle3 = Turtle('turtle3', 'green')
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        turtle1.move()
        turtle2.move()
        turtle3.move()
        check_collision(turtle1, turtle2)
        check_collision(turtle1, turtle3)
        check_collision(turtle2, turtle3)
        rate.sleep()