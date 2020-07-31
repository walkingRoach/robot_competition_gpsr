#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import cv2

from walk import Leg
from config import Config

NODE_NAME = "test_go_pos_node"

place_dict = ['livingroom', 'chicken', 'hallway', 'bedroom']

if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=False)
    rospy.Rate(10)
    config = Config()
    leg = Leg()
    for room in place_dict:
        place_list = config.room_dict[room]
        for place in place_list:
            rospy.loginfo('will go {}'.format(place))
            if leg.move(config.nav_dict[place], config.move_time_limit):
                rospy.loginfo('success reach target pos')
            else:
                rospy.loginfo('fail reach target pos')
            rospy.sleep(5)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")

    cv2.destroyAllWindows()

