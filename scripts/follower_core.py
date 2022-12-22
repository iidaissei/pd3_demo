#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from detection_msgs.msg import BoundingBoxes

# PID制御のパラメータ
lKp = rospy.get_param("/follower_core/lKp")
lKi = rospy.get_param("/follower_core/lKi")
lKd = rospy.get_param("/follower_core/lKd")
aKp = rospy.get_param("/follower_core/aKp")
aKi = rospy.get_param("/follower_core/aKi")
aKd = rospy.get_param("/follower_core/aKd")
max_linear  = rospy.get_param("/follower_core/max_linear")
min_linear  = rospy.get_param("/follower_core/min_linear")
max_angular = rospy.get_param("/follower_core/max_angular")
min_angular = rospy.get_param("/follower_core/min_angular")

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


class HumanFollower():
    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self.laserCB)
        rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.yoloCB)
        self.twist_pub = rospy.Publisher(pub_twist_name, Twist, queue_size = 10)
        self.front_dist = 0.0
        self.twist = Twist()
        self.cx = self.cy = None
        self.last_err_x = self.last_err_y = 0.0

    def laserCB(self, scan):
        # 前方90度の範囲の最小値を求める
        self.fornt_dist = min(scan.ranges[180:520])

    def yoloCB(self, bb_msg):
        if not bb_msg.bounding_boxes:
            self.cx = self.cy = None
        else:
            # BoundingBoxesのClass"human"から重心座標を算出する
            bb_human = bb_msg.bounding_boxes[0]
            self.cx = round(bb_human.xmin + (bb_human.xmax - bb_human.xmin)/2)
            self.cy = round(bb_human.ymin + (bb_human.ymax - bb_human.ymin)/2)

    def pidUpdate(self):
        # 重心座標と目標座標の偏差を求める
        err_x = self.cx - target_px[0]
        err_y = target_px[1] - self.cy
        print ("err_x: %d, err_y: %d" % (err_x, err_y))
        # 並進PID制御の算出
        pid_l = (lKp * err_y + lKi * (err_y + self.last_err_y) + lKd * (err_x - self.last_err_y))
        pid_l = round(pid_l, 3)
        if min_linear >= pid_l:
            self.twist.linear.x = min_linear
        elif max_linear <= pid_l:
            self.twist.linear.x = max_linear
        else:
            self.twist.linear.x = pid_l
        # 旋回PID制御の算出
        pid_a = (aKp * err_x + aKi * (err_x + self.last_err_x) + aKd * (err_y - self.last_err_x))
        pid_a = round(pid_a, 3)
        if min_angular >= pid_a:
            self.twist.angular.z = min_angular
        elif max_angular <= pid_a:
            self.twist.angular.z = max_angular
        else:
            self.twist.angular.z = pid_a
        self.last_err_x = err_x
        self.last_err_y = err_y
        print ("pid_l: %f, pid_a: %f" % (pid_l, pid_a))
        print(self.twist.linear.x, self.twist.angular.z)

    def pdUpdate(self):
        # 重心座標と目標座標の偏差を求める
        err_x = self.cx - target_px[0]
        err_y = target_px[1] - self.cy
        print ("err_x: %d, err_y: %d" % (err_x, err_y))
        # 並進PD制御の算出
        pd_l = (lKp * err_y + lKd * (err_x - self.last_err_y))
        pd_l = round(pd_l, 3)
        if min_linear >= pd_l:
            self.twist.linear.x = min_linear
        elif max_linear <= pd_l:
            self.twist.linear.x = max_linear
        else:
            self.twist.linear.x = pd_l
        # 旋回PD制御の算出
        pd_a = (aKp * err_x + aKd * (err_y - self.last_err_x))
        pd_a = round(pd_a, 3)
        if min_angular >= pd_a:
            self.twist.angular.z = min_angular
        elif max_angular <= pd_a:
            self.twist.angular.z = max_angular
        else:
            self.twist.angular.z = pd_a
        self.last_err_x = err_x
        self.last_err_y = err_y
        print ("pd_l: %f, pd_a: %f" % (pd_l, pd_a))
        print(self.twist.linear.x, self.twist.angular.z)

    def followCtrl(self):
        while not rospy.is_shutdown():
            if self.front_dist < safety_dist:
                self.twist.linear.x = self.twist.angular.z = 0.0
                rospy.loginfo("!! Emergency stop !!")
            if self.cx == None:
                self.twist.linear.x = self.twist.angular.z = 0.0
                # rospy.loginfo("No human detected...")
            elif range_xmin <= self.cx <= range_xmax and range_ymin <= self.cy <= range_ymax: 
                # self.pidUpdate()
                self.pdUpdate()
                # rospy.loginfo("Human detected...")
            else:
                self.twist.linear.x = self.twist.angular.z = 0.0
                # rospy.loginfo("Out of range...")
                pass
            self.twist_pub.publish(self.twist)
            rospy.sleep(0.1)


if __name__=='__main__':
    rospy.init_node('follower_core', anonymous = True)
    ci = HumanFollower()
    ci.followCtrl()
