#!/usr/bin/env python
import time
import threading

import rospy
import smach

from sensor_msgs.msg import LaserScan


class WaitDoorOpen(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['waiting', 'door_opened'])

        self.robot = robot

        self.mutex = threading.Lock()
        self.door_opened = False
        # Avoid lidar mess up sometimes, so we accumulate time
        self.robot_frontage_empty_time = 0

        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.callback)

    def callback(self, data):
        self.mutex.acquire()

        front_lidar_data = data.ranges[-5:] + data.ranges[:5]
        robot_frontage_is_empty = all(i > 1
                                      for i in front_lidar_data)

        if robot_frontage_is_empty:
            self.robot_frontage_empty_time += 1
            if self.robot_frontage_empty_time > 30:
                self.door_opened = True

        self.mutex.release()

    def execute(self, ud):
        # wait for a maximum of 30 seconds
        for i in range(0, 300):
            self.mutex.acquire()
            if self.door_opened:
                self.subscriber.unregister()
                return 'door_opened'

            self.mutex.release()

            rospy.sleep(.1)

        return 'waiting'
