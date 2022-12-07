#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from detection_msgs.msg import BoundingBoxes

# lidarと人の目標距離
target_num = 0.5 # [m]
# target_numをpixelに変換（lidar_to_imnageノードの変換処理に依存）
# 少数点になるとcvの方でエラーになるので丸める
# target_px[x座標, y座標]
target_px = [250, round(target_num * 100)] # paramから取得するようにしたい（依存関係あるので）
Kp = 1000
range_xmin = 200
range_xmax = 300
range_ymin = 100
range_ymax = 250


class FollowCtrl():
    def __init__(self):
        rospy.Subscriber('yolov5/detections', BoundingBoxes, self.yoloCB)
        # self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.twist_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size = 1)
        self.twist = Twist()
        self.bb = []
        self.cx = self.cy = None

    def yoloCB(self, msg):
        self.bb = msg.bounding_boxes
        if not self.bb:
            self.cx = self.cy = None
        else:
            # BoundingBoxesのClass"human"から重心座標を算出する
            bb_human = self.bb[0]
            self.cx = round(bb_human.xmin + (bb_human.xmax - bb_human.xmin)/2)
            self.cy = round(bb_human.ymin + (bb_human.ymax - bb_human.ymin)/2)
            # print(self.cx, self.cy)

    def pController(self):
        # 重心座標と目標座標の差
        while not rospy.is_shutdown():
            if self.cx == None:
                self.twist.linear.x = self.twist.angular.z = 0.0
                # rospy.loginfo("No human detected...")
                pass
            elif range_xmin <= self.cx <= range_xmax and range_ymin <= self.cy <= range_ymax: 
                # 人の誤認識にある程度対応するため
                self.twist.linear.x = self.twist.angular.z = 0.0
                err = self.cx - target_px[0]
                self.twist.linear.x = 0.2
                self.twist.angular.z = -float(err)/Kp
                rospy.loginfo("human detected...")
                pass
            else:
                self.twist.linear.x = self.twist.angular.z = 0.0
                rospy.loginfo("Out of range...")
            self.twist_pub.publish(self.twist)
            rospy.sleep(0.1)


if __name__=='__main__':
    rospy.init_node('follower_core', anonymous = True)
    ci = FollowCtrl()
    ci.pController()
    rospy.spin()
