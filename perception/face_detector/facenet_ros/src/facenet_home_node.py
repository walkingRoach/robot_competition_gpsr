#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os

import rospy
import cv_bridge
import roslib
roslib.load_manifest('darknet_ros2')
from darknet_ros_msgs.msg import *
from facenet_home import *
from std_msgs.msg import Int16,Int32,String


NODE_NAME = "facenet_home"
BOX_THRESHOLD = 0.7


class ImageRecognition:
    def __init__(self, sub_bounding_boxes_topic, database, model_path):
        self.recognition = Recognition(model_path, self.get_imgs_db(database))
        self.bridge = cv_bridge.CvBridge()
        #pub = rospy.Publisher('veriface', String, queue_size=1)
        rospy.Subscriber(sub_bounding_boxes_topic, BoundingBoxes, self.callback_bounding_boxes, queue_size=1)

    def callback_bounding_boxes(self, bounding_boxes_msg):
        for box in bounding_boxes_msg.bounding_boxes:
            if box.Class == "face" and box.probability > BOX_THRESHOLD:
                self.handle_face(box, bounding_boxes_msg)
            elif box.Class == "people" and box.probability > BOX_THRESHOLD:
                self.handle_face(box, bounding_boxes_msg)

    def handle_face(self, boxes, msg):
        x1 = boxes.xmin
        x2 = boxes.xmax
        y1 = boxes.ymin
        y2 = boxes.ymax
        pub = rospy.Publisher('/xf/tts/words',String, queue_size=1)  #veriface
        img_msg = msg.image
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        res = self.recognition.identify(cv_image[y1:y2, x1:x2])
        if res:
            print(res.name)
            pub.publish("你好，老朋友")
            pub.publish("你好，老朋友")
        else:
            print("stranger")

            pub.publish("你好，我不认识你")
            pub.publish("你好，我不认识你")
            

    def handle_people(self, box, msg):
        pass

    def get_imgs_db(self, database):
        imgs_db = []

        for f in os.listdir(database):
            if f.endswith("jpg"):
                name = f[:-4]
                img = cv2.imread(database + f)
                img = cv2.resize(img, (160, 160), interpolation=cv2.INTER_CUBIC)
                imgs_db.append((img, name))

        return imgs_db

if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=False)
    model_path = rospy.get_param("~model_path")
    database = rospy.get_param("~database")
    subscribe_image_topic = rospy.get_param("~subscribe_image_topic")
    subscribe_bounding_boxes_topic = rospy.get_param("~subscribe_bounding_boxes_topic")
    action_detection_topic = rospy.get_param("~action_detection_topic")

    ir = ImageRecognition(subscribe_bounding_boxes_topic, database, model_path)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"

    cv2.destroyAllWindows()
