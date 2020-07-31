# -*- coding: utf-8 -*-

import smach
import rospy


class BackInit(smach.State):
    def __init__(self, robot):
        self.robot = robot

        smach.State.__init__(self, outcomes=['finished'])

        rospy.loginfo('start find people')

    def execute(self, ud):
        self.robot.nav_by_place_name('back_pose')

        return 'finished'