#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2 as cv
# import numpy as np
import rospy
import cv_bridge
from sensor_msgs.msg import Image
from detection_msgs.msg import BoundingBoxes

# lidarと人の目標距離
target_num = 0.5 # [m]
# target_numをpixelに変換（lidar_to_imnageノードの変換処理に依存）
# 少数点になるとcvの方でエラーになるので丸める
# paramから取得するようにしたい（依存関係あるので）
target_px = [250, 250 -round(target_num * 100)]


class CreateImage():
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        rospy.Subscriber('yolov5/detections', BoundingBoxes, self.yoloCB)
        rospy.Subscriber('/scan_to_image', Image, self.showImage)
        self.image = None
        self.bb = []
        self.cx = None
        self.cy = None

    def yoloCB(self, msg):
        self.bb = msg.bounding_boxes
        if not self.bb:
            self.cx = None
            self.cy = None
        else:
            # BoundingBoxesのClass"human"から重心座標を算出する
            bb_human = self.bb[0]
            self.cx = round(bb_human.xmin + (bb_human.xmax - bb_human.xmin)/2)
            self.cy = round(bb_human.ymin + (bb_human.ymax - bb_human.ymin)/2)
            # print(self.cx, self.cy)

    # 目標座標の描画処理
    def plotTargetPoint(self):
        cv.circle(self.image,
                  center = (target_px[0], target_px[1]),
                  radius = 5,
                  color = (255, 0, 0),
                  thickness = 2)

    # 重心座標の描画処理
    def plotCenterPoint(self):
        cv.circle(self.image,
                  center = (self.cx, self.cy),
                  radius = 10,
                  color = (0, 0, 255),
                  thickness = 2)

    # ウィンドウへの表示処理
    def showImage(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
        self.plotTargetPoint()
        if self.cx == None:
            # rospy.loginfo("No human detected...")
            pass
        else:
            self.plotCenterPoint()
        cv.imshow('follow_human', self.image)
        cv.waitKey(1)


if __name__=='__main__':
    rospy.init_node('follower_imgshow', anonymous = True)
    ci = CreateImage()
    rospy.spin()
