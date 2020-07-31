#!/usr/bin/python
# -*- coding: utf-8 -*-


import rospy
import body
import cv2
import numpy as np


NODE_NAME = "test_perception_node"

if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=False)
    rospy.Rate(10)
    model_path = rospy.get_param("~model_path")
    datebase = rospy.get_param("~database")

    subscribe_image_topic = rospy.get_param("~subscribe_image_topic")
    subscribe_bounding_boxes_topic = rospy.get_param("~subscribe_bounding_boxes_topic")
    action_detection_topic = rospy.get_param("~action_detection_topic")

    # ir = Eye(subscribe_bounding_boxes_topic, model_path, datebase)
    eye = Eye(action_detection_topic, model_path, datebase, True)

    people_boxes, peopel_img, people_name = eye.detect_obj_message()

    if np.all(peopel_img != None):
        cv2.imwrite('/home/ubuntu/Desktop/1.jpg', peopel_img)
        # cv2.waitKey(3)
    else:
        rospy.loginfo('please input image!')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")

    cv2.destroyAllWindows()

