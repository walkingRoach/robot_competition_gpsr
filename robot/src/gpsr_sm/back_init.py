# -*- coding: utf-8 -*-

import smach
import rospy


class BackInit(smach.State):
    def __init__(self, robot):
        self.robot = robot

        smach.State.__init__(self, outcomes=['finish'])

        rospy.loginfo('start find people')

    def execute(self, ud):
        rospy.loginfo('go back')

        self.robot.nav_by_place_name('back_pose')
        # rospy.sleep(10)

        return 'finish'