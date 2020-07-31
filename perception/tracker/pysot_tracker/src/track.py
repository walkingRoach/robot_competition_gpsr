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

import cv2
import torch
import numpy as np

# import sys
# sys.path.append(os.path.join(os.getcwd(), 'pysot'))

from pysot.core.config import cfg
from pysot.models.model_builder import ModelBuilder
from pysot.tracker.tracker_builder import build_tracker

torch.set_num_threads(1)

PEOPLE_MIN_DISTANCE = 1.5
OBJ_MIN_DISTANCE = 0.65
MAX_DISTANCE = 4.0
MAX_LINEAR_SPEED = 0.6
MIN_LINEAR_SPEED = 0.15
MAX_ROTATION_SPEED = 0.75

k_linear_speed = (MAX_LINEAR_SPEED - MIN_LINEAR_SPEED) / (MAX_DISTANCE - PEOPLE_MIN_DISTANCE)
h_linear_speed = MIN_LINEAR_SPEED - k_linear_speed * PEOPLE_MIN_DISTANCE

k_rotation_speed = 0.002
h_rotation_speed_left = 1.20   # 1.20
h_rotation_speed_right = 1.36

# should resize depend on input image size
ERROR_OFFSET_X_left1 = 200
ERROR_OFFSET_X_left2 = 500
ERROR_OFFSET_X_right1 = 550
ERROR_OFFSET_X_right2 = 1080


class PysotTracker:
    def __init__(self):
        config_path = rospy.get_param("~config")
        snapshot = rospy.get_param("~snapshot")
        self.visualize = rospy.get_param("~visualize")

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

        end_sub = rospy.Subscriber('/tracker/end', Bool, self.stop_track, queue_size=1)
        box_sub = rospy.Subscriber('/tracker/enable', Box, self.begin_track, queue_size=1)

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
        self.is_select = rospy.get_param("~is_select")
        self.last_distance = 0
        self.first_frame = False

        # speed and angle paras
        self.linear_speed = 0
        self.rotation_speed = 0

        # track target
        self.target = None

        # if start depth is 0
        self.start_depth = 0

    def stop_track(self, msg):
        rospy.loginfo('follow stop {}'.format(msg))
        self.is_tracking = False
        self.bRenewROI = False
        self.is_select = False
        self.last_distance = 0
        self.box = None
        self.target = None
        self.enable_update = False
        self.enable_get_depth = False
        self.enable_update = False
        self.first_frame = False
        self.linear_speed = 0
        self.rotation_speed = 0
        self.start_depth = 0
        self.speed_pub.publish(Twist())
        cv2.destroyAllWindows()

    def begin_track(self, msg):
        # bbox = list(msg)
        rect = (msg.xmin, msg.ymin, msg.width, msg.height)
        # print(rect)
        self.target = msg.obj_name
        rospy.loginfo('track target is {}'.format(self.target))
        self.box = rect
        # img = self.cv_bridge.imgmsg_to_cv2(msg.image, "bgr8")

        self.is_tracking = True
        self.bRenewROI = True
        self.start_depth = 0
        self.first_frame = True

    def imageCb(self, msg):
        if self.is_tracking is False and self.is_select is False:
            return
        img = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

        # except:
        #     rospy.ERROR('fail change to cv2')
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
            depth_list.append(depth_array[int(self.bbox[1]+self.bbox[3]/3), int(self.bbox[0] + 2*self.bbox[2]/3)])
            depth_list.append(depth_array[int(self.bbox[1]+2*self.bbox[3]/3), int(self.bbox[0] + 2*self.bbox[2]/3)])

            for i in range(len(depth_list)):
                if (depth_list[i] > 400.0) and (depth_list[i] < 8000.0):
                    depth_num += 1
                    depth_sum += depth_list[i]

            if depth_sum == 0 and self.start_depth == 0:
                rospy.loginfo('start depth is zero')
                twist = Twist()
                twist.linear.x = 0.1
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = 0
                traker.speed_pub.publish(twist)
                rospy.sleep(1)

            self.start_depth += 1
            try:
                depth_distance = depth_sum / depth_num / 1000.0
            except ZeroDivisionError as e:
                if self.target == 'people' and self.last_distance > PEOPLE_MIN_DISTANCE:
                    depth_distance = self.last_distance
                elif self.target != 'people' and self.last_distance > OBJ_MIN_DISTANCE:
                    depth_distance = self.last_distance
                else:
                    depth_distance = 0

            if ((depth_distance - self.last_distance) > 1.4) and (self.first_frame is False):
                rospy.loginfo('track fail')
                depth_distance = self.last_distance

            if self.first_frame:
                self.first_frame = False
                rospy.loginfo('init first frame')

            self.last_distance = depth_distance

            if (self.target == 'people') and (depth_distance > PEOPLE_MIN_DISTANCE):
                rospy.loginfo('people test')
                self.linear_speed = depth_distance * k_linear_speed + h_linear_speed
            elif self.is_select and (depth_distance > PEOPLE_MIN_DISTANCE):
                rospy.loginfo('select is test')
                self.linear_speed = depth_distance * k_linear_speed + h_linear_speed
            elif (self.target != 'people') and (self.is_select is False) and (depth_distance > OBJ_MIN_DISTANCE):
                self.linear_speed = depth_distance * k_linear_speed + h_linear_speed
            else:
                rospy.loginfo('people fail')
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

            rospy.loginfo('linear_speed:{} rotation_speed:{} center_x:{}'.format(self.linear_speed, self.rotation_speed, center_x))

            rospy.loginfo('depth_distance is {}'.format(depth_distance))

            self.enable_get_depth = False
            twist = Twist()
            twist.linear.x = self.linear_speed
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.rotation_speed

            if traker.is_tracking:
                rospy.loginfo('send twist')
                traker.speed_pub.publish(twist)
                pass

 
if __name__ == '__main__':
    rospy.init_node("pysot_tracker", anonymous=False)

    traker = PysotTracker()

    stop_count = 0

    while True:
        if traker.is_tracking:
            pass
    # rospy.spin()

            # rospy.loginfo('waiting tracking')

            # using voice control stop

            # using time stop by speak
            # if (traker.linear_speed == 0) and (traker.rotation_speed == 0):
            #     stop_count += 1
            # else:
            #     stop_count = 0
            #
            # if stop_count > 20:
            #     rospy.loginfo('stop track')
            #     # traker.is_tracking = False
            #     traker.stop_track()
            #     cv2.destroyAllWindows()
