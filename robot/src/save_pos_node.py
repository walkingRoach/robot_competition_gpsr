#!/usr/bin/python
# -*- coding: utf-8

import os
import rospy

from geometry_msgs.msg import PoseStamped

NODE_NAME = "save_pos_node"


class SavePos:
    def __init__(self, save_path):
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.callback)
        self.fp = open(save_path, "a")

    def callback(self, msg):
        position = msg.pose.position
        quaternion = msg.pose.orientation
        self.save_txt(position, quaternion)

    def save_txt(self, position, quaternion):
        self.fp.write('position x:{}, y:{}, z:{}\n'.format(position.x, position.y, position.z))
        self.fp.write('quaternion x: {}, y:{}, z:{}, w:{}\n'.format(quaternion.x,
                                                                    quaternion.y,
                                                                    quaternion.z,
                                                   quaternion.w))
        self.fp.write('\n')


if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=False)
    rospy.Rate(10)

    save_path = rospy.get_param("~save_path")

    save_pos = SavePos(save_path)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS save pos module")



