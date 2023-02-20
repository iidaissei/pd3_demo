#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import math
import rospy
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge, CvBridgeError

# 縮小サイズを取得. 1[pixel] = 0.01[mm]pixel
disc_size = rospy.get_param("/laser_to_image/disc_size")
# disc_factor
disc_factor = 1/disc_size
# Max Lidar Range
max_lidar_range = rospy.get_param("/laser_to_image/max_range")
# max_lidar_rangeとdisc_factorを使って画像サイズを設定する
image_size = int(max_lidar_range*2*disc_factor)
rospy.set_param("/laser_to_image/image_size", image_size)
# 画像を表示するか否かのフラグ
imshow_flg = rospy.get_param("/laser_to_image/img_show")

class laser_to_image():
    def __init__(self):
        self.joy_sub = rospy.Subscriber('/scan',LaserScan,self.cloud_to_image_callback)
        self.pub = rospy.Publisher('/laser_to_image',Image, queue_size = 10)
        self.bridge = CvBridge()

    def cloud_to_image_callback(self, scan):
        maxAngle = scan.angle_max
        minAngle = scan.angle_min
        angleInc = scan.angle_increment
        maxLength = scan.range_max
        ranges = scan.ranges
        num_pts = len(ranges)
        xy_scan = np.zeros((num_pts, 2))
        # 3チャンネルの白色ブランク画像を作成
        blank_image = np.zeros((image_size, image_size, 3), dtype=np.uint8) + 255
        # rangesの距離・角度から全ての点をXYに変換する処理
        for i in range(num_pts):
            # 範囲内かを判定
            if (ranges[i] > max_lidar_range) or (math.isnan(ranges[i])):
                pass
            else:
                # 角度とXY座標の算出処理
                angle = minAngle + float(i)*angleInc
                xy_scan[i][1] = float(ranges[i]*math.cos(angle))
                xy_scan[i][0] = float(ranges[i]*math.sin(angle))

        # ブランク画像にプロットする処理
        for i in range(num_pts):
            pt_x = xy_scan[i, 0]
            pt_y = xy_scan[i, 1]
            if (pt_x < max_lidar_range) or (pt_x > -1 * (max_lidar_range-disc_size)) or (pt_y < max_lidar_range) or (pt_y > -1 * (max_lidar_range-disc_size)):
                pix_x = int(math.floor((pt_x + max_lidar_range) * disc_factor))
                pix_y = int(math.floor((max_lidar_range - pt_y) * disc_factor))
                if (pix_x > image_size) or (pix_y > image_size):
                    print("Error")
                else:
                    blank_image[pix_y, pix_x] = [0, 0, 0]

        # CV2画像からROSメッセージに変換してトピックとして配布する
        img = self.bridge.cv2_to_imgmsg(blank_image, encoding="bgr8")
        self.pub.publish(img)

        # 画像の表示処理. imshow_flgがTrueの場合は表示する
        if imshow_flg:
            cv2.imshow('laser_to_image', blank_image), cv2.waitKey(3)
            # 更新のため一旦消す
            blank_image = np.zeros((image_size,image_size,3))
        else:
            pass


if __name__=='__main__':
    try:
        rospy.init_node('laser_to_image', anonymous = True)
        laser_to_image = laser_to_image()
        rospy.spin()
    except rospy.ROSException:
        pass
