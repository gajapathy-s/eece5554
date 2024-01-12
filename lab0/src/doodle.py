#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

robot_x = 0
robot_y = 0

def turtle_pos_callback(Pose):
    global robot_x, robot_y
    rospy.loginfo("Robot_x = %f, Robot_y = %f\n", Pose.x, Pose.y)
    robot_x = Pose.x
    robot_y = Pose.y

def path(vel, pub, rate, l_x, l_y, condition):
    while not rospy.is_shutdown() and condition():
        vel.linear.x = l_x
        vel.linear.y = l_y
        pub.publish(vel)
        rate.sleep()
        
    rospy.loginfo("Robot Reached Destination")
    rospy.logwarn("Stopping robot")

def turtle():
    rospy.init_node("move_turtle", anonymous="True")
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/turtle1/pose", Pose, turtle_pos_callback)
    rate = rospy.Rate(10)
    vel = Twist()

    right = lambda: robot_x <= 8
    oup = lambda: robot_y <= 7
    left = lambda: robot_x >= 6
    tup = lambda: robot_y <= 8

    path(vel, pub, rate, 1, 0, right)
    path(vel, pub, rate, 0, 1, oup)
    path(vel, pub, rate, -1, 0, left)
    path(vel, pub, rate, 0, 1, tup)
    path(vel, pub, rate, 1, 0, right)

    

if __name__ == "__main__":
    try:
        turtle()
    except rospy.ROSInterruptException:
        pass

