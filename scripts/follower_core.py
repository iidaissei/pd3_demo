#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from detection_msgs.msg import BoundingBoxes

# PID制御のパラメータ
xKp = rospy.get_param("/follower_core/xKp")
xKi = rospy.get_param("/follower_core/xKi")
xKd = rospy.get_param("/follower_core/xKd")
yKp = rospy.get_param("/follower_core/yKp")
yKi = rospy.get_param("/follower_core/yKi")
yKd = rospy.get_param("/follower_core/yKd")

# 追従関連のパラメータ
range_xmin     = rospy.get_param("/follower_core/range_xmin")
range_xmax     = rospy.get_param("/follower_core/range_xmax")
range_ymin     = rospy.get_param("/follower_core/range_ymin")
range_ymax     = rospy.get_param("/follower_core/range_ymax")
safety_dist    = rospy.get_param("/follower_core/safety_dist")
pub_twist_name = rospy.get_param("/follower_core/pub_twist_name")

# 目標座標(target_px)の生成に使用するパラメータ
disc_size      = rospy.get_param("/laser_to_image/disc_size")
target_dist    = rospy.get_param("/follower_core/target_dist")
# 目標座標の生成. [x座標, y座標]
target_px = [250, 250 - round(target_dist/disc_size)]


class FollowCtrl():
    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self.laserCB)
        rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.yoloCB)
        self.twist_pub = rospy.Publisher(pub_twist_name, Twist, queue_size = 1)
        self.front_dist = 999
        self.twist = Twist()
        self.bb = []
        self.cx = self.cy = None
        self.last_error_x = self.last_error_y = 0.0
        self.delta_err_x = self.delta_err_y = 0.0

    def laserCB(self, scan):
        ranges = scan.ranges
        self.front_dist = ranges[round(len(ranges)/2)]

    def yoloCB(self, bb_msg):
        self.bb = bb_msg.bounding_boxes
        if not self.bb:
            self.cx = self.cy = None
        else:
            # BoundingBoxesのClass"human"から重心座標を算出する
            bb_human = self.bb[0]
            self.cx = round(bb_human.xmin + (bb_human.xmax - bb_human.xmin)/2)
            self.cy = round(bb_human.ymin + (bb_human.ymax - bb_human.ymin)/2)
            # print(self.cx, self.cy)

    def pidUpdate(self):
        # 重心座標と目標座標の偏差を求める
        err_x = self.cx - target_px[0]
        err_y = self.cy - target_px[0]
        delta_err_x = err_x - self.last_err_x
        delta_err_y = err_y - self.last_err_y
        # PIDの算出
        # Pの算出
        p_x = xKp * err_x
        p_y = xKp * err_y
        # Iの算出
        i_x += err_x * 
        pid_x = (xKp * err_x + xKi * (err_x + last_err_x) + xKd * (err_x - self.last_err_x))
        pid_y = (yKp * err_y + yKi * (err_y + last_err_y) + yKd * (err_y - self.last_err_y))
        self.twist.linear.x  = pid_x
        self.twist.angular.z = pid_y
        self.last_err_x = err_x
        self.last_err_y = err_y
        print(self.twist.linear.x)
        print(self.twist.angular.z)

    def execute(self):
        while not rospy.is_shutdown():
            if self.front_dist < safety_dist:
                self.twist.linear.x = self.twist.angular.z = 0.0
                rospy.loginfo("!! Emergency stop !!")
            if self.cx == None:
                # self.twist.linear.x = self.twist.angular.z = 0.0
                rospy.loginfo("No human detected...")
            elif range_xmin <= self.cx <= range_xmax and range_ymin <= self.cy <= range_ymax: 
                # self.twist.linear.x = self.twist.angular.z = 0.0
                self.pidUpdate()
                rospy.loginfo("Human detected...")
            else:
                # self.twist.linear.x = self.twist.angular.z = 0.0
                rospy.loginfo("Out of range...")
            self.twist_pub.publish(self.twist)
            # rospy.sleep(0.1)


if __name__=='__main__':
    rospy.init_node('follower_core', anonymous = True)
    ci = FollowCtrl()
    ci.execute()
    rospy.spin()
