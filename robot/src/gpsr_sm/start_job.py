# -*- coding: utf-8 -*-

import smach
import rospy


class InitRobot(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['exit_state', 'fail'])

        self.robot = robot

    def execute(self, ud):
        rospy.loginfo('start to gpsr')
        rospy.sleep(2)
        self.robot.nav_by_place_name('init_pose')
        self.robot.speak_wav('hello')

        return 'exit_state'
