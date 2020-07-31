#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from grasp import Arm
from config import Config
from perception import Eye
from walk import Leg
from follower import ObjTracker

NODE_NAME = "test_capture_node"


def close_obj_via_follow(obj):
    # 开始跟踪
    if tracker.init_track(obj):
        # 进行强制定时办法
        # rospy.sleep(5)
        # 订阅速度节点
        stop_close = False
        stop_count = 0
        while not stop_close:
            twist = rospy.wait_for_message('/cmd_vel', Twist)
            if twist.linear.x < 0.012 and twist.angular.z < 0.01:
                stop_count += 1
            else:
                stop_count = 0

            if stop_count > 10:
                rospy.loginfo('stop close')
                stop_follow_pub.publish(False)
                return True
    else:
        return False


if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=False)
    rospy.Rate(10)

    config = Config()
    arm = Arm(config)
    rospy.sleep(18)
    rospy.loginfo('初始化结束')
    eye = Eye(Config.yolo_action_topic, Config.facenet_model, Config.database_path, Config.enable_facenet)
    leg = Leg()
    tracker = ObjTracker(eye)

    stop_follow_pub = rospy.Publisher('/tracker/end', Bool, queue_size=1)

    if close_obj_via_follow('greenTea'):
        # leg.approach_obj(0.1, 0.0)
        arm.grasp_obj(70)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")

    # cv2.destroyAllWindows()
