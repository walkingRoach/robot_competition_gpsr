#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def save_color_depth_image(i, save_depth=False):
    color_image = rospy.wait_for_message('/camera/color/image_raw', Image, 10)
    if color_image is None:
        rospy.loginfo('fail get color image')
    else:
        color_image = bridge.imgmsg_to_cv2(color_image, "bgr8")

        cv2.imshow('color_img', color_image)

    if save_depth is True:
        depth_image = rospy.wait_for_message('/camera/aligned_depth_to_color/image_raw', Image)

        if depth_image is None:
            rospy.loginfo('fail get depth image')
        else:
            depth_image = bridge.imgmsg_to_cv2(depth_image, desired_encoding='passthrough')

        # cv2.imshow('depth image', depth_image)

    k = cv2.waitKey(10)
    # rospy.loginfo('saving image!')
    if k == ord('s'):
        rospy.loginfo('save image')
        i += 1
        cv2.imwrite(os.path.expanduser('~/robot_ros/yolo_dataset/supermarket/') + 'obj_' + str(i) + '.jpg', color_image)
        if save_depth is True:
            cv2.imwrite('/home/ubuntu/robot_ros/src/robot/image/depth/' + str(i) + '.png', depth_image)
    return i


if __name__ == '__main__':
    rospy.init_node('save_image', anonymous=False)

    rate = rospy.Rate(20)

    bridge = CvBridge()

    i = 0
    while not rospy.is_shutdown():
        i = save_color_depth_image(i)
        rate.sleep()

    rospy.spin()
