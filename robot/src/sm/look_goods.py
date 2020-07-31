# -*- coding: utf-8 -*-

import smach
import rospy


class LookingGoods(smach.State):
    def __init__(self, robot):
        self.robot = robot

        smach.State.__init__(self, outcomes=['finished', 'fail'])

        rospy.loginfo('start looking goods')
        self.get_job_flag = False
        self.try_again = True

        self.obj_dict = {
            'bathCream':'沐浴露',
            'milkTea':'奶茶',
            'napkin':'纸巾',
            'oreo':'奥利奥',
            'pepsi':'百事可乐',
            'chips':'薯片',
            'green':'绿茶',
            'herbal':'王老吉',
            'porridge':'粥',
            'biscuits':'饼干'
        }

    def execute(self, ud):
        # jobs = self.robot.memory.get_jobs()
        self.robot.nav_by_place_name('take_goods')

        obj_list = self.robot.memory.obj_list
        rospy.loginfo(obj_list)

        self.get_job_flag = True

        while not self.get_job_flag:
            for obj in obj_list:
                rospy.loginfo('get command %s'%self.obj_dict[obj])
                # obj_poses = self.robot.find_obj_poses(obj, direction='left')
                # if obj_poses is not None:
                #     for pose in obj_poses:
                #         self.robot.nav_by_poes(pose)
                if self.robot.close_obj_via_follow(obj, stop_twist_x=0.01):
                    self.robot.speak('发现 {}'.format(self.obj_dict[obj]))
                    if self.robot.could_grasp:
                        if self.robot.arm.grasp(50):
                            self.get_job_flag = True
                            self.robot.leg.approach_obj(-0.2, 0)
                                # 暂时只抓一个
                            return 'finished'
                else:
                    rospy.loginfo('can not find obj')
                    if self.try_again:
                        self.try_again = False
                        self.robot.leg.approach_obj(0.1, 0)
                    else:
                        self.robot.speak("对不起我找不到物品")
                        return 'finished'

        return 'finished'