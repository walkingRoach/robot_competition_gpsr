#!/usr/bin/python
# -*- coding: utf-8

from nav_msgs.msg import Odometry
import rospy

from geometry_msgs.msg import PoseStamped

NODE_NAME = "save_pos_node"


class SavePos:
    def __init__(self):
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_call_back)

    def odom_call_back(self, msg):
        odom_msg = Odometry()
        odom_msg = msg
        position = odom_msg.pose.pose.position.x

        # rospy.loginfo('odom {}'.format(odom_msg))
        rospy.loginfo('position {}'.format(position))


if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=False)
    rospy.Rate(10)

    # save_path = rospy.get_param("~save_path")

    save_pos = SavePos()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS save pos module")



