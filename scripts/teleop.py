#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys

import select
import termios
import tty
settings = termios.tcgetattr(sys.stdin) 

def key_input():
    # referred https://gist.github.com/jasonrdsouza/1901709?permalink_comment_id=2734411 for this function

    try:
        # tty.setraw(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno()) 
        clist, val1, val2 = select.select([sys.stdin], [], [], 0.05)
        if clist:
            val = sys.stdin.read(1)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            return val
        
        else:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            return ''
        
    except KeyboardInterrupt:
        handle_exit_event()

def handle_exit_event():
    rospy.signal_shutdown("Imminent Shutdown requested")

def navigate(input):
    if input == 'w':
        forward()
    elif input == 's':
        backward()
    elif input == 'a':
        left()
    elif input == 'd':
        right()
    elif input == 'e':
        # stop()
        rospy.signal_shutdown("Shutdown requested")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        exit(0)
    else:
        print("Invalid Input")

def forward():
    twist = Twist()
    twist.linear.x = 1
    twist.angular.z = 0
    pub.publish(twist)

def backward():
    twist = Twist()
    twist.linear.x = -1
    twist.angular.z = 0
    pub.publish(twist)

def left(): 
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = 1
    pub.publish(twist)

def right():
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = -1
    pub.publish(twist)

def stop():
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = 0
    pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('jackal_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
 

    rate = rospy.Rate(1000) # 1000hz
    rospy.loginfo("Welcome to Jackal Teleop")
    print("WASD to move the robot. E to exit.")
    while not rospy.is_shutdown():
        char_in = key_input()
        if char_in:
            print(char_in)
            navigate(char_in)
        rate.sleep()
    rospy.spin()
