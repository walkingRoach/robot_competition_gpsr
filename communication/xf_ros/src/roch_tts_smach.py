#!/usr/bin/env python
# -*- coding: utf-8 -*-

#--------------------------------------------------
# China 2017 ROS node of state machine for GPSR
#
# author: Vance Wu
# date: 17/07/27
#--------------------------------------------------

# import sys
import rospy

import smach
import smach_ros

#import roslib
import actionlib

from subprocess import call

from roch_tts.msg import RochTTSAction
from roch_tts.msg import RochTTSGoal
from roch_tts.msg import RochTTSFeedback
from roch_tts.msg import RochTTSResult

# sys.path.append(roslib.packages.get_pkg_dir('common_pkg') + '/scripts/common')

# from common_import import *
# from common_function import *

class init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit1'])

    def execute(self, userdata):
        return 'exit1'

class TTS(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit1'])

    def execute(self, userdata):
        self.text_to_speech('这是你爸爸做的smash程序')

        return 'exit1'

    def text_to_speech(self, speech_str):
        rospy.loginfo(speech_str)

        tts_action_client = actionlib.SimpleActionClient('roch_tts_action', RochTTSAction)
        tts_action_client.wait_for_server()

        goal = RochTTSGoal()
        goal.roch_tts_goal = speech_str
        tts_action_client.send_goal(goal)

        finished = tts_action_client.wait_for_result()
        if finished:
            state = tts_action_client.get_state()
            rospy.loginfo("Action finished, state: %i", state)
            rospy.loginfo("Action finished, %s", tts_action_client.get_result())
        else:
            rospy.loginfo("Action did not finish before the time out.")

def main():
    rospy.init_node("roch_tts_client")

    sm_tts = smach.StateMachine(outcomes = ['failed', 'successed'])

    rospy.loginfo('Start state machine')
    with sm_tts:
        smach.StateMachine.add('init', init(), transitions={'exit1':'TTS'})
        smach.StateMachine.add('TTS', TTS(), transitions={'exit1':'successed'})

    sis = smach_ros.IntrospectionServer("sm_tts", sm_tts, '/SM_ROOT')
    sis.start()
    outcome = sm_tts.execute()

    while not rospy.is_shutdown():
        rospy.spin()

    sis.stop()

if __name__ == "__main__":
    main()