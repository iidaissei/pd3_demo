#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

safety_dist    = rospy.get_param("/follower_core/safety_dist")
pub_twist_name = "/cmd_vel_mux/input/emergency"
# pub_twist_name = "/cmd_vel_mux/input/teleop"

class EmergencyStop():
    def __init__(self):
        sub = rospy.Subscriber("/scan", LaserScan, self.laserCB)
        self.twist_pub = rospy.Publisher(pub_twist_name, Twist, queue_size = 3)
        self.twist = Twist()
        self.rate = rospy.Rate(50)
        self.front_dist = 0.0

    def laserCB(self, scan):
        # 前方90度の範囲の最小値を求める
        # front_dist = min(scan.ranges[180:540])
        self.front_dist = scan.ranges[359]

    def execute(self):
        while not rospy.is_shutdown():
            if self.front_dist < safety_dist:
                self.twist.linear.x = self.twist.angular.z = 0.0
                self.twist_pub.publish(self.twist)
                rospy.loginfo("Emergency stop!!")
            else:
                pass
            self.rate.sleep()


if __name__=='__main__':
    rospy.init_node('emergency_stop', anonymous = True)
    es = EmergencyStop()
    es.execute()
