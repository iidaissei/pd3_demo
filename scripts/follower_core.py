#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from detection_msgs.msg import BoundingBoxes

# パラメータの読み込み
Kp = rospy.get_param("/follower_core/Kp")
range_xmin     = rospy.get_param("/follower_core/range_xmin")
range_xmax     = rospy.get_param("/follower_core/range_xmax")
range_ymin     = rospy.get_param("/follower_core/range_ymin")
range_ymax     = rospy.get_param("/follower_core/range_ymax")
target_dist    = rospy.get_param("/follower_core/target_dist")
put_twist_name = rospy.get_param("/follower_core/put_twist_name")
disc_size      = rospy.get_param("/laser_to_image/disc_size")
# target_numをpixelに変換. target_px[x座標, y座標]
target_px = [250, 250 - round(target_dist/disc_size)]


class FollowCtrl():
    def __init__(self):
        rospy.Subscriber('yolov5/detections', BoundingBoxes, self.yoloCB)
        self.twist_pub = rospy.Publisher(put_twist_name, Twist, queue_size = 1)
        # self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        # self.twist_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size = 1)
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
                rospy.loginfo("human detected...")
                self.twist.linear.x = self.twist.angular.z = 0.0
                err = self.cx - target_px[0]
                self.twist.linear.x = 0.2
                self.twist.angular.z = -float(err)/Kp
                pass
            else:
                rospy.loginfo("Out of range...")
                self.twist.linear.x = self.twist.angular.z = 0.0
            self.twist_pub.publish(self.twist)
            rospy.sleep(0.1)


if __name__=='__main__':
    rospy.init_node('follower_core', anonymous = True)
    ci = FollowCtrl()
    ci.pController()
    rospy.spin()
