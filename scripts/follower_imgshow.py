#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2 as cv
import rospy
import cv_bridge
from sensor_msgs.msg import Image
from detection_msgs.msg import BoundingBoxes

# ロボットと追従対象者との目標距離を設定
target_dist = rospy.get_param("/follower_control/target_dist")
disc_size   = rospy.get_param("/laser_to_image/disc_size")
# target_distからピクセル座標系で目標座標を生成する
target_px = [250, 250 - round(target_dist/disc_size)]


class FollowerImshow():
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.sub_bb = rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.yoloCB)
        self.sub_img = rospy.Subscriber('/laser_to_image', Image, self.showImage)
        self.cx = self.cy = None
        self.image = None

    def yoloCB(self, bb_msg):
        if not bb_msg.bounding_boxes:
            self.cx = self.cy = None
        else:
            # BoundingBoxesのClass"human"から重心座標を算出する
            bb_human = bb_msg.bounding_boxes[0]
            self.cx = round(bb_human.xmin + (bb_human.xmax - bb_human.xmin)/2)
            self.cy = round(bb_human.ymin + (bb_human.ymax - bb_human.ymin)/2)

    # 中心座標の描画処理
    def plotRobotPoint(self):
        cv.circle(self.image,
                  center = (250, 250),
                  radius = 5, 
                  color = (0, 255, 0),
                  thickness = -1)

    # 目標座標の描画処理
    def plotTargetPoint(self):
        cv.circle(self.image,
                  center = (target_px[0],
                  target_px[1]),
                  radius = 10, 
                  color = (255, 0, 0),
                  thickness = 2)

    # 重心座標の描画処理
    def plotCentroidPoint(self):
        cv.circle(self.image,
                  center = (self.cx, self.cy),
                  radius = 8,
                  color = (0, 0, 255),
                  thickness = -1)

    # ウィンドウへの表示処理
    def execute(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
        self.plotTargetPoint()
        self.plotRobotPoint()
        if self.cx == self.cy == None:
            pass
        else:
            self.plotCentroidPoint()
        cv.imshow('human_follower', self.image)
        cv.waitKey(1)


if __name__=='__main__':
    rospy.init_node('follower_imgshow', anonymous = True)
    try:
        fi = FollowerImshow()
        rospy.spin()
    except rospy.ROSException:
        pass
