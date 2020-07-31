#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import rospy
import cv_bridge
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import os
import cv2
import torch
import numpy as np

# import sys
# sys.path.append(os.path.join(os.getcwd(), 'pysot'))

from pysot.core.config import cfg
from pysot.models.model_builder import ModelBuilder
from pysot.tracker.tracker_builder import build_tracker

torch.set_num_threads(1)

MIN_DISTANCE = 1.0
MAX_DISTANCE = 6.0
MAX_LINEAR_SPEED = 0.4
MIN_LINEAR_SPEED = 0.1
MAX_ROTATION_SPEED = 0.75

k_linear_speed = (MAX_LINEAR_SPEED - MIN_LINEAR_SPEED) / (MAX_DISTANCE - MIN_DISTANCE)
h_linear_speed = MIN_LINEAR_SPEED - k_linear_speed * MIN_DISTANCE

k_rotation_speed = 0.002
h_rotation_speed_left = 1.2
h_rotation_speed_right = 1.36

# should resize depend on input image size
ERROR_OFFSET_X_left1 = 200
ERROR_OFFSET_X_left2 = 600
ERROR_OFFSET_X_right1 = 680
ERROR_OFFSET_X_right2 = 1080


class PysotTracker:
    def __init__(self):
        config_path = rospy.get_param("~config")
        snapshot = rospy.get_param("~snapshot")
        self.visualize = rospy.get_param("~visualize")
        self.is_select = rospy.get_param("~is_select")

        cfg.merge_from_file(config_path)
        cfg.CUDA = torch.cuda.is_available()
        device = torch.device('cuda')

        # create model
        model = ModelBuilder()

        # load model
        model.load_state_dict(torch.load(snapshot,
                                         map_location=lambda storage, loc: storage.cpu()))
        model.eval().to(device)

        # build tracker
        self.tracker = build_tracker(model)

        self.end_pub = rospy.Publisher('/tracker/end', Bool, queue_size=1)

        image_sub_ = rospy.Subscriber('/camera/color/image_raw', Image, self.imageCb, queue_size=1)
        depth_sub_ = rospy.Subscriber('camera/depth/image_rect_raw', Image, self.depthCb, queue_size=1)

        self.speed_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.cv_bridge = cv_bridge.CvBridge()
        # track params
        self.is_tracking = False
        self.bRenewROI = False
        self.box = None
        self.enable_get_depth = False
        self.enable_update = False
        self.bbox = None
        self.last_distance = 0

        # speed and angle paras
        self.linear_speed = 0
        self.rotation_speed = 0

    def stop_track(self):
        self.is_tracking = False
        self.bRenewROI = False
        self.is_select = False
        self.last_distance = 0
        self.box = None
        self.enable_update = False
        self.enable_get_depth = False
        self.enable_update = False
        self.linear_speed = 0
        self.rotation_speed = 0
        self.end_pub.publish(True)

    def begin_track_by_select(self, msg):
        bbox = list(msg)
        print(bbox)
        self.box = bbox
        img = self.cv_bridge.imgmsg_to_cv2(msg.image, "bgr8")
        self.is_tracking = True
        self.bRenewROI = True

    def imageCb(self, msg):
        if self.is_tracking is False and self.is_select is False:
            return
        # print(msg)
        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        rospy.loginfo('ok')
        if self.is_select:
            self.box = cv2.selectROI('track', img, False, False)
            rospy.loginfo(self.box)
            self.bRenewROI = True
            self.is_select = False
            self.is_tracking = True

        if self.bRenewROI:
            self.tracker.init(img, self.box)
            self.bRenewROI = False
            self.enable_update = True

        if self.enable_update:

            outputs = self.tracker.track(img)
            self.bbox = list(map(int, outputs['bbox']))
            if self.visualize:
                cv2.rectangle(img, (self.bbox[0], self.bbox[1]),
                              (self.bbox[0]+self.bbox[2], self.bbox[1]+self.bbox[3]),
                              (0, 255, 0), 3)
                cv2.imshow('track', img)
                cv2.waitKey(10)
            self.enable_get_depth = True

    def depthCb(self, msg):
        if self.is_tracking is False:
            return

        depth_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        depth_array = np.array(depth_img, dtype=np.float32)

        if self.enable_get_depth:
            depth_list = []
            depth_num = 0
            depth_sum = 0
            depth_list.append(depth_array[int(self.bbox[1]+self.bbox[3]/3), int(self.bbox[0] + self.bbox[2]/3)])
            depth_list.append(depth_array[int(self.bbox[1]+self.bbox[3]/2), int(self.bbox[0] + self.bbox[2]/2)])
            depth_list.append(depth_array[int(self.bbox[1]+2*self.bbox[3]/3), int(self.bbox[0] + self.bbox[2]/3)])

            for i in range(len(depth_list)):
                if (depth_list[i] > 400.0) and (depth_list[i] < 8000.0):
                    depth_num += 1
                    depth_sum += depth_list[i]

            try:
                depth_distance = depth_sum / depth_num / 1000.0
            except ZeroDivisionError as e:
                if self.last_distance > MIN_DISTANCE:
                    depth_distance = self.last_distance
                else:
                    depth_distance = 0

            self.last_distance = depth_distance

            rospy.loginfo('depth_distance is {}'.format(depth_distance))

            if depth_distance > MIN_DISTANCE:
                self.linear_speed = depth_distance * k_linear_speed + h_linear_speed
            else:
                self.linear_speed = 0

            if self.linear_speed > MAX_LINEAR_SPEED:
                self.linear_speed = MAX_LINEAR_SPEED

            center_x = self.bbox[0] + self.bbox[2] / 2
            if center_x < ERROR_OFFSET_X_left1:
                self.rotation_speed = MAX_ROTATION_SPEED
            elif (center_x > ERROR_OFFSET_X_left1) and (center_x < ERROR_OFFSET_X_left2):
                self.rotation_speed = -k_rotation_speed * center_x + h_rotation_speed_left
            elif (center_x > ERROR_OFFSET_X_right1) and (center_x < ERROR_OFFSET_X_right2):
                self.rotation_speed = -k_rotation_speed * center_x + h_rotation_speed_right
            elif center_x > ERROR_OFFSET_X_right2:
                self.rotation_speed = -MAX_ROTATION_SPEED
            else:
                self.rotation_speed = 0

            rospy.loginfo('linear_speed:{} rotation_speed:{}'.format(self.linear_speed, self.rotation_speed))

            rospy.loginfo('depth_distance is {}'.format(depth_distance))

            self.enable_get_depth = False


if __name__ == '__main__':
    rospy.init_node("pysot_tracker", anonymous=False)

    traker = PysotTracker()

    stop_count = 0

    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x = traker.linear_speed
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = traker.rotation_speed

        if traker.is_tracking:
            traker.speed_pub.publish(twist)

