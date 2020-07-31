#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from perception import Eye
from memory import TargetPeople, Memory
from communication import Ear

import cv2
import signal
import os
import numpy as np
import fitz

NODE_NAME = "test_face_node"


class InputTimeoutError(Exception):
    rospy.loginfo('timeout')
    pass


def interrupted(signum, frame):
    raise InputTimeoutError


def make_pdf():
    pdf_file = os.path.expanduser("~/Desktop/people.pdf")

    people_path = os.path.expanduser('~/robot_ros/src/robot/image/people')

    image_list = os.listdir(people_path)
    input_file = os.path.expanduser("~/robot_ros/test.pdf")
    file_handle = fitz.open(input_file)

    for image_name in image_list:

        barcode_file = os.path.join(people_path, image_name)

        # define the position (upper-right corner)
        image_rectangle = fitz.Rect(20, 20, 300, 300)

        test_point = fitz.Point(20, 320)
        # retrieve the first page of the PDF

        page = file_handle.newPage()

        # add the image
        page.insertImage(image_rectangle, filename=barcode_file)

        page.insertText(test_point, image_name.split('.')[0])

    file_handle.save(pdf_file)


def clean_img():
    people_path = os.path.expanduser('~/robot_ros/src/robot/image/people')

    file_list = os.listdir(people_path)

    for file_name in file_list:
        os.remove(os.path.join(people_path, file_name))


if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=False)
    rospy.Rate(10)
    model_path = rospy.get_param("~model_path")
    datebase = rospy.get_param("~database")

    subscribe_image_topic = rospy.get_param("~subscribe_image_topic")
    subscribe_bounding_boxes_topic = rospy.get_param("~subscribe_bounding_boxes_topic")
    action_detection_topic = rospy.get_param("~action_detection_topic")
    listen_action_topic = rospy.get_param("~listen_action_topic")

    # ir = Eye(subscribe_bounding_boxes_topic, model_path, datebase)
    eye = Eye(action_detection_topic, model_path, datebase, True)
    ear = Ear(listen_action_topic)
    memory = Memory()

    rospy.loginfo('start face detect')
    i, try_times = 0, 4
    detect_stop = False
    face_feature, face_img = None, None
    while i < try_times and not detect_stop:
        signal.signal(signal.SIGALRM, interrupted)
        signal.alarm(5)

        try:
            face_feature, face_img = eye.detect_face()
            detect_stop = True
        except InputTimeoutError:
            i += 1

        signal.alarm(0)

    # 得到信息的内容
    rospy.loginfo('start get name and obj_detect')
    people_name, obj_name = ear.get_name_and_object()

    rospy.loginfo('start save people')
    clean_img()
    face_path = os.path.expanduser("~/robot_ros/src/robot/image/people/") + str(people_name) + '_' + str(obj_name)+'.jpg'
    cv2.imwrite(face_path, face_img)

    people = TargetPeople(people_name, obj_name, face_feature, face_path)

    memory.add_people(people)
    people.print_people()

    make_pdf()
    # 开始测试对比人脸
    rospy.loginfo('start detect face and identify face')
    detect_stop = False
    while i < try_times and not detect_stop:
        signal.signal(signal.SIGALRM, interrupted)
        signal.alarm(5)

        try:
            face_feature, face_img = eye.detect_face()
            detect_stop = True
        except InputTimeoutError:
            i += 1

        signal.alarm(0)
    name = None
    best = 10.0
    for db in memory.people_with_job_list:
        dist = (face_feature - db.peopleFace).norm().item()
        if dist < best:
            best = dist
            name = db.peopleName
            rospy.loginfo('test a face simlilar {}'.format(best))
    if best < 0.95:
        rospy.loginfo(name)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")

    cv2.destroyAllWindows()

