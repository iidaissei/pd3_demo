#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

safety_dist    = rospy.get_param("/follower_core/safety_dist")
pub_twist_name = rospy.get_param("/follower_core/pub_twist_name")


class EmergencyStop():
    def __init__(self):
        sub = rospy.Subscriber("/scan", LaserScan, self.twistPub)
        self.twist_pub = rospy.Publisher(pub_twist_name, Twist, queue_size = 3)
        self.twist = Twist()
        self.twist.linear.x = self.twist.angular.z = 0.0

    def twistPub(self, scan):
        # min_dist = scan.ranges[359]
        # 小さすぎる値を取り除く処理。
        filtered_ranges = [i for i in scan.ranges if not i < 0.05]
        min_dist = min(filtered_ranges)
        if min_dist < safety_dist:
            # print (min_dist)
            self.twist_pub.publish(self.twist)
            rospy.loginfo("!!Emergency Stop!!")
        else:
            pass


if __name__=='__main__':
    rospy.init_node('emergency_stop', anonymous = True)
    es = EmergencyStop()
    rospy.spin()
