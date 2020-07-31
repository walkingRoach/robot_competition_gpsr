# -*- coding: utf-8 -*-

import smach
import rospy

# import sys

# reload(sys)
# sys.setdefaultencoding('utf-8')


class MeetPeople(smach.State):
    def __init__(self, robot):
        self.robot = robot

        self.get_name_flag = False

        smach.State.__init__(self, outcomes=['finished', 'fail'])

        rospy.loginfo('start meet people')

    def execute(self, ud):
        # meet people and identify them
        rospy.loginfo('start to meet people and identify them')
        # self.robot.leg.approach_obj(-3.0, 0.0)
        self.robot.nav_by_place_name('back_find_people')
        people_poses = None
        try_times = 0

        while people_poses is None and try_times < 2:
            dircetion = self.robot.get_place_direction('back_find_people')
            people_poses = self.robot.config.back_pose_dict
            # people_poses = self.robot.find_people_poses(direction=dircetion, place='back_find_people')
            if people_poses is None:
                try_times += 1
                self.robot.leg.approach_obj(-0.1, 0)
                rospy.loginfo('fail get pose')
            else:
                for people_pose in people_poses:
                    if self.robot.nav_by_poes(people_pose):
                        # self.robot.leg.approach_obj(0.35, 0.1)
                        name = self.robot.identify_people()

                        rospy.loginfo('get name is {}'.format(name))
                        if name is None:
                            rospy.loginfo('I can not identify you')
                            continue

                        people = self.robot.memory.get_obj_by_name(name)
                        if people is not None:
                            self.robot.speak('你好{}你是要{}对吗'.format(people.peopleName, people.peopleJob))

        return 'finished'
