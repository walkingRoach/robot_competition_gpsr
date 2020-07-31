# -*- coding: utf-8 -*-

import smach
import rospy


class InitRobot(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['exit_state'])

        self.robot = robot

    def execute(self, ud):
        # 前进三,四米,并进行自我介绍
        rospy.loginfo('start init')
        # while not self.robot.open_door():
        rospy.loginfo('wait start node')
        # self.robot.nav_by_place_name('init_pose')

        return 'exit_state'
