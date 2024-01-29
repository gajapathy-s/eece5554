#!/usr/bin/env python3

import rospy
from gps_driver.msg import Customgps

def gps_callback(data):
    cmsg=Customgps
    rospy.loginfo()
def gps_listener():
    rospy.init_node('gps_listener', anonymous=True)
    rospy.Subscriber('chatter', Customgps, gps_callback)
    rospy.spin()

if __name__ == '__main__':
    gps_listener()


















