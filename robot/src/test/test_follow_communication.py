#!/usr/bin/python
# -*- coding: utf-8
import sys
reload(sys)
sys.setdefaultencoding('utf-8')

import rospy
from perception import Eye
from std_msgs.msg import Bool
from follower import ObjTracker
from geometry_msgs.msg import Twist

NODE_NAME = "test_follow_node"

def velCb(msg):
    global stop_num
    global stop_follow
    twist = Twist()
    twist = msg
    rospy.loginfo('twist augular z {}'.format(twist.angular.z))
    if twist.angular.z == 0:
        stop_num += 1
    else:
        stop_num = 0

    if stop_num > 20:
        rospy.loginfo('stop follow')
        stop_follow_pub.publish(False)
        rospy.sleep(1)
        stop_follow_pub.publish(False)


def follow(follow_obj='face'):
    if tracker.start_track_face():
        rospy.loginfo('啦啦啦')
    else:
        rospy.loginfo('can not find face')


if __name__ == '__main__':
    stop_follow_pub = rospy.Publisher('/tracker/end', Bool, queue_size=1)

    rospy.init_node(NODE_NAME, anonymous=False)
    rospy.Rate(10)

    model_path = rospy.get_param("~model_path")
    datebase = rospy.get_param("~database")

    subscribe_image_topic = rospy.get_param("~subscribe_image_topic")
    subscribe_bounding_boxes_topic = rospy.get_param("~subscribe_bounding_boxes_topic")
    action_detection_topic = rospy.get_param("~action_detection_topic")

    eye = Eye(action_detection_topic, model_path, datebase, False)
    tracker = ObjTracker(eye)
    stop_num = 0
    stop_follow = 0
    follow('people')
    vel_sub = rospy.Subscriber('/cmd_vel', Twist, velCb, queue_size=10)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")


