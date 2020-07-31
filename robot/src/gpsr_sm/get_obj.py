# -*- coding: utf-8 -*-

import smach
import rospy


class GetObj(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['finish'],
                             input_keys=['get_obj_in'])
        self.robot = robot

    def execute(self, ud):
        rospy.loginfo('start get obj')
        job_one = ud.get_obj_in

        # rospy.loginfo(job_one.print_job())
        # 解析任务, 抓东西给我
        if job_one.m_type == 1:
            # form somewhere take something to me
            current_pose = self.robot.get_current_pose()   # 得到当前位置

            # face_feature, face_img = self.robot.remember_people()

            self.robot.nav_by_place_name(job_one.get_room)  # 先去某地
            # dircetion = self.robot.get_place_direction(job_one.get_room)

            # obj_pose = self.robot.find_obj_poses(job_one.aobject, dircetion)   # 这个需要进一步测试
            # if obj_pose is not None:
            # if self.robot.nav_by_poes(obj_pose):
            find_obj = False
            if not self.robot.close_obj_via_follow(job_one.aobject):
                rospy.loginfo('fail find obj')
                self.robot.speak('对不起我不能找到物品')
            else:
                find_obj = True
                self.robot.mouth.speak_via_pub('find {}'.format(job_one.aobject))

            # self.robot.grasp()    # 抓取
            if self.robot.could_grasp and find_obj:
                self.robot.arm.grasp(self.robot.get_height_via_name(job_one.get_room))
                self.robot.leg.approach_obj(-0.2, 0)

            self.robot.nav_by_poes(current_pose)
        elif job_one.m_type == 2:
            # from somewhere get something to somebody which in the somewhere
            self.robot.nav_by_place_name(job_one.get_room)
            # direction = self.robot.get_place_direction(job_one.get_room)
            # obj_pose = self.robot.find_obj_poses(job_one.aobject, direction)
            # if obj_pose is not None:
            # if self.robot.nav_by_poes(job_one.aobject):
            find_obj = False
            if not self.robot.close_obj_via_follow(job_one.aobject):
                rospy.loginfo('fail find obj')
                self.robot.speak('对不起我找不到物体')
            else:
                find_obj = True
                self.robot.speak('find {}'.format(job_one.aobject))

            if self.robot.could_grasp and find_obj:

                self.robot.arm.grasp(self.robot.get_height_via_name(job_one.get_room))

            self.robot.nav_by_place_name(job_one.target_room)

            direction = self.robot.get_place_direction(job_one.target_room)

            people_poses = None
            try_times = 0

            while people_poses is None and try_times < 2:
                people_poses = self.robot.find_people_poses(direction, job_one.target_room)
                if people_poses is not None:
                    for i in range(len(people_poses)):
                        if (i+1) == len(people_poses):
                            self.robot.nav_by_poes(people_poses[i])
                            self.robot.speak('你好{}你的{}'.format(job_one.target_person, job_one.aobject))
                            if self.robot.could_grasp:
                                self.robot.arm.give_obj(200)
                            return 'finish'
                        else:
                            self.robot.nav_by_poes(people_poses[i])
                            people_name = self.robot.get_name()
                            if people_name == job_one.target_person:
                                self.robot.speak('你好{}你的{}'.format(job_one.target_person, job_one.aobject))
                            else:
                                self.robot.speak('这个东西可能不是给你的')
                            return 'finish'
                else:
                    try_times += 1
                    self.robot.leg.approach_obj(0.1, 0.0)
                    rospy.loginfo('fail find people, and test again')
        else:
            # from someshere get something to somewhere
            self.robot.nav_by_place_name(job_one.get_room)

            # obj_pose = self.robot.find_obj_poses(job_one.aobject)

            # self.robot.nav_by_poes(obj_pose)
            find_obj = False

            if not self.robot.close_obj_via_follow(job_one.aobject):
                rospy.loginfo('fail find obj')
                self.robot.speak('对不起我找不到物品')
            else:
                find_obj = True
                self.robot.mouth.speak_via_pub('find {}'.format(job_one.aobject))

            if self.robot.could_grasp and find_obj:
                self.robot.arm.grasp(self.robot.get_height_via_name(job_one.get_room))
                self.robot.leg.approach_obj(-0.2, 0)
            self.robot.nav_by_place_name(job_one.target_room)

            if self.robot.could_grasp and find_obj:
                height = self.robot.get_height_via_name(job_one.target_room) + 130
                self.robot.arm.give_obj(height)

        return 'finish'
