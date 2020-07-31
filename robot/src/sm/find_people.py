# -*- coding: utf-8 -*-

import smach
import rospy


class FindPeople(smach.State):
    def __init__(self, robot):
        self.robot = robot

        smach.State.__init__(self, outcomes=['finished', 'fail'])

        rospy.loginfo('start find people')

    def execute(self, ud):
        rospy.loginfo('go to find people')
        # 进行找人并进行语音
        self.robot.nav_by_place_name('find_people')
        self.robot.speak_wav('hello')
        rospy.sleep(2)
        # self.robot.speak_wav('hello')
        people_poses = None

        try_times = 0
        while people_poses is None and try_times < 2:
            direction = self.robot.get_place_direction('find_people')
            # people_poses = self.robot.find_people_poses(direction, 'find_people')
            people_poses = self.robot.config.find_pose_dict
            self.robot.clean_img()
            if people_poses is not None:
                for people_pose in people_poses:
                    self.robot.nav_by_poes(people_pose)
                    self.robot.leg.approach_obj(0.0, -0.1)

                    people = self.robot.add_people()

                    people.print_people()
                    self.robot.speak('你好{},你想拿{}'.format(people.peopleName, people.peopleJob))
                    self.robot.memory.add_people(people)
            else:
                try_times += 1
                self.robot.leg.approach_obj(-0.1, 0)
                rospy.loginfo('fail find people ')

        self.robot.make_pdf()
        rospy.sleep(3)
        return 'finished'
