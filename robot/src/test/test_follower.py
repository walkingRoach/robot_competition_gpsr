#!/usr/bin/env python
import sys

import cv2
import rospy
import cv_bridge
from std_msgs.msg import Bool, Header
from sensor_msgs.msg import Image


def build_box(box, img_msg, obj_name):
    bbox = Box()
    header = Header()
    bbox.obj_name = str(obj_name)
    header.stamp = rospy.Time.now()
    width = box[1] - box[0]
    height = box[3] - box[2]
    scale = 0.1
    bbox.xmin = box[0] + int(scale * width)
    bbox.ymin = box[2] + int(scale * height)
    bbox.width = int((1 - scale) * width)
    bbox.height = int((1 - scale) * height)
    return bbox


class ObjTracker:
    def __init__(self, eye):
        self.eye = eye
        self.bridge = cv_bridge.CvBridge()

        self.end_sub = rospy.Subscriber('/tracker/end', Bool, self.end_cb)
        self.box_pub = rospy.Publisher('/tracker/enable', Box, queue_size=1)
        # self.image = rospy.wait
        self.image = self.img = None
        self.is_tracking = True
        self.select = False
        # self.box = {}
        self.box = Box()
        self.origin_x = 0
        self.origin_y = 0
        self.done_flag = False
        # cv2.namedWindow('image')

    def end_cb(self, msg):
        self.is_tracking = False

    def obj_from_eye(self):
        self.image = rospy.wait_for_message("/camera/color/image_raw", Image)

        self.img = self.bridge.imgmsg_to_cv2(self.image, 'bgr8')
        cv2.setMouseCallback('image', self.on_mouse)
        # rospy.loginfo(self.box)
        cv2.imshow('image', self.img)
        cv2.waitKey(1)

    def init_track(self, target):
        rospy.loginfo("init tracking")
        self.is_tracking = True
        # select_done = False
        boxes, image, _ = self.eye.detect_obj_message(target)
        if boxes is None:
            return False

        box = build_box(boxes[0], self.bridge.cv2_to_imgmsg(image, 'bgr8'), target)
        rospy.loginfo('send success')
        rospy.loginfo(box.obj_name)
        self.box_pub.publish(box)
        rospy.loginfo("Starting %s tracking" % target)
        # cv2.destroyWindow('image')
        return True

    def start_track_face(self):
        boxes, face_img = self.eye.detect_face_with_identify()
        if boxes is None:
            return False

        box = build_box(boxes[0], self.bridge.cv2_to_imgmsg(face_img, 'bgr8'), 'face')

        rospy.loginfo('send success')
        rospy.loginfo(box.obj_name)
        self.box_pub.publish(box)
        rospy.loginfo("Starting tracking face")
        # cv2.destroyWindow('image')
        return True


def main():
    from perception import Eye
    from config import Config

    rospy.init_node('obj_tracking')

    eye = Eye(Config.yolo_action_topic, Config.facenet_model, Config.database_path, Config.enable_facenet)
    #
    tracker = ObjTracker(eye)
    tracker.init_track(target="people")
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        r.sleep()
        if not tracker.is_tracking:
            rospy.loginfo("tracking finished!")
            break


if __name__ == '__main__':
    sys.exit(main())
