# -*- coding: utf-8 -*-

import smach
import rospy


class GoRoom(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['finish'],
                             input_keys=['go_room_in'])

        self.robot = robot

    def execute(self, ud):
        rospy.loginfo('start go room and search for an obj_detect')

        job_two = ud.go_room_in
        # rospy.loginfo(job_two.print_job())
        # 解析任务
        # self.robot.nav_by_place_name(job_two.room)  # 先去某地

        if self.robot.search_by_place_name(job_two.room, job_two.aobject):
            rospy.loginfo('success find obj_detect')
        else:
            rospy.loginfo('fail find obj_detect')

        return 'finish'
