# -*- coding: utf-8 -*-

import smach
import rospy


class FindPeople(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['finish'],
                             input_keys=['find_people_in'])

        self.robot = robot

    def execute(self, ud):
        rospy.loginfo('start find people and talk or follow')

        job_three = ud.find_people_in
        # rospy.loginfo(job_three.print_job())

        if job_three.m_type == 1:
            # from somewhere find a person tell something
            self.robot.nav_by_place_name(job_three.room)

            direction = self.robot.get_place_direction(job_three.room)

            people_pose = self.robot.find_people_pose(direction, rule='random', place=job_three.room)
            try_times = 0

            while people_pose is None and try_times < 2:
                self.robot.leg.approach_obj(-0.1, 0.0)

                people_pose = self.robot.find_people_pose(direction, rule='random', place=job_three.room)

            self.robot.nav_by_poes(people_pose)

            self.robot.leg.approach_obj(0.2, 0.0)

            self.robot.talk_to_person(job_three.talk)
        elif job_three.m_type == 2:
            # from somewhere find a person and ask something, notice that you need to connect to the network
            self.robot.nav_by_place_name(job_three.room)

            direction = self.robot.get_place_direction(job_three.room)

            people_pose = self.robot.find_people_pose(direction, rule='random')
            try_times = 0
            # 暂时只尝试一次
            while people_pose is None and try_times < 2:
                self.robot.leg.approach_obj(-0.1, 0.0)

                people_pose = self.robot.find_people_pose(direction, rule='random', place=job_three.room)

                if people_pose is None:
                    try_times += 1

            self.robot.nav_by_poes(people_pose)

            self.robot.leg.approach_obj(0.2, 0.0)

            if self.robot.ask_to_person():
                rospy.loginfo('success answer a question')
            else:
                rospy.loginfo('fail answer a question')
        elif job_three.m_type == 3:
            # from somewhere find a person and follow him, notice that you need to connect to the network
            rospy.loginfo('room is {}'.format(job_three.room))
            self.robot.nav_by_place_name(job_three.room)

            # people_pose = self.robot.find_people_pose(rule='random')
            if not self.robot.follow_with_voice('people'):
                self.robot.leg.approach_obj(-0.2, 0)
                self.robot.follow_with_voice('people')
        return 'finish'
