# -*- coding: utf-8 -*-

import smach
import rospy


class GetJob(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,
                             outcomes=['mission1', 'mission2', 'mission3'],
                             output_keys=['command'])
        self.robot = robot

    def execute(self, ud):
        rospy.loginfo('start get command')
        # 进行语音识别
        self.robot.speak('你有什么任务可以给我吗')
        rospy.sleep(3)
        mission_type, mission_ctype, mission, confirm_dict = self.robot.ear.get_res()
        mission_type, mission = self.robot.confirm_job(mission_type, mission_ctype, mission, confirm_dict)

        # 传递参数
        ud.job = mission
        # rospy.loginfo(mission.print_job())
        res = self.robot.my_aiml.respond(mission.print_job())
        self.robot.speak(str(res).replace('_', ' '))
        self.robot.make_gpsr_pdf(mission)
        if mission_type == 'one':
            return 'mission1'  # get_obj
        elif mission_type == 'two':
            return 'mission2'  # go_room
        else:
            return 'mission3'  # find_people
