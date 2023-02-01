#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from pd3_demo.msg import Follower
from geometry_msgs.msg import Twist
from detection_msgs.msg import BoundingBoxes

# 制御関連のパラメータ
lKp = rospy.get_param("/follower_control/lKp")
lKi = rospy.get_param("/follower_control/lKi")
lKd = rospy.get_param("/follower_control/lKd")
aKp = rospy.get_param("/follower_control/aKp")
aKi = rospy.get_param("/follower_control/aKi")
aKd = rospy.get_param("/follower_control/aKd")
max_linear  = rospy.get_param("/follower_control/max_linear")
min_linear  = rospy.get_param("/follower_control/min_linear")
max_angular = rospy.get_param("/follower_control/max_angular")
min_angular = rospy.get_param("/follower_control/min_angular")
pub_twist_name = rospy.get_param("/follower_control/pub_twist_name")

# 人を認識する範囲のパラメータ
range_xmin     = rospy.get_param("/follower_control/range_xmin")
range_xmax     = rospy.get_param("/follower_control/range_xmax")
range_ymin     = rospy.get_param("/follower_control/range_ymin")
range_ymax     = rospy.get_param("/follower_control/range_ymax")

# 目標座標(target_px)の生成に使用するパラメータ
disc_size      = rospy.get_param("/laser_to_image/disc_size")
target_dist    = rospy.get_param("/follower_control/target_dist")
# 目標座標の生成. [x座標, y座標]
target_px = [250, 250 - round(target_dist/disc_size)]


class HumanFollower():
    def __init__(self):
        self.bb_sub = rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.followCtrl)
        self.twist_pub = rospy.Publisher(pub_twist_name, Twist, queue_size = 10)
        self.last_err_x = self.last_err_y = 0.0
        self.cx = self.cy = None
        self.twist = Twist()
        self.pd3_pub = rospy.Publisher("/pd3_topic", Follower, queue_size = 10)
        self.pd3_data = Follower()

    def pidUpdate(self):
        # 重心座標と目標座標の偏差を求める
        err_x = self.cx - target_px[0]
        err_y = target_px[1] - self.cy
        # print ("err_x: %d, err_y: %d" % (err_x, err_y))
        # 並進PID制御の算出
        pid_l = (lKp * err_y + lKi * (err_y + self.last_err_y) + lKd * (err_y - self.last_err_y))
        pid_l = round(pid_l, 3)
        if min_linear >= pid_l:
            self.twist.linear.x = min_linear
        elif max_linear <= pid_l:
            self.twist.linear.x = max_linear
        else:
            self.twist.linear.x = pid_l
        # 旋回PID制御の算出
        pid_a = (aKp * err_x + aKi * (err_x + self.last_err_x) + aKd * (err_x - self.last_err_x))
        pid_a = round(pid_a, 3)
        if min_angular >= pid_a:
            self.twist.angular.z = min_angular
        elif max_angular <= pid_a:
            self.twist.angular.z = max_angular
        else:
            self.twist.angular.z = pid_a
        self.last_err_x = err_x
        self.last_err_y = err_y
        ### 実験データの収集########################
        # self.pd3_data.err_x = err_x
        # self.pd3_data.err_y = err_y
        # self.pd3_pub.publish(self.pd3_data)
        ############################################
        print("pid_linear: %f, pid_angular: %f" % (pid_l, pid_a))
        print(self.twist.linear.x, self.twist.angular.z)

    def followCtrl(self, bb_msg):
        if not bb_msg.bounding_boxes:
            self.cx = self.cy = None
            # self.twist.linear.x = self.twist.angular.z = 0.0
            rospy.loginfo("No human detected...")
        else:
            # BoundingBoxesのClass"human"から重心座標を算出する
            bb_human = bb_msg.bounding_boxes[0]
            self.cx = round(bb_human.xmin + (bb_human.xmax - bb_human.xmin)/2)
            self.cy = round(bb_human.ymin + (bb_human.ymax - bb_human.ymin)/2)
            if range_xmin <= self.cx <= range_xmax and range_ymin <= self.cy <= range_ymax: 
                self.pidUpdate()
                rospy.loginfo("Human detected...")
            else:
                self.twist.linear.x = self.twist.angular.z = 0.01
                rospy.loginfo("Out of range...")
                pass
            self.twist_pub.publish(self.twist)
            rospy.sleep(0.01)


if __name__=='__main__':
    rospy.init_node('follower_control', anonymous = True)
    ci = HumanFollower()
    rospy.spin()
