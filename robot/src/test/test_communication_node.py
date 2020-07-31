#!/usr/bin/python
# -*- coding: utf-8

import sys
reload(sys)
sys.setdefaultencoding('utf-8')

import rospy
from communication import Mouth, Ear

import aiml
import os

import fitz

NODE_NAME = "test_communication_node"


def sigintHandle():
    rospy.loginfo('Terminating the node...')
    get_job_flag = True
    ear.cancel_asr()
    rospy.signal_shutdown('haha')


def confirm_job(mission_type, mission_ctype, mission, confirm_dict):
    speak = None
    if mission_type == 'two':
        # 这个最简单,使用local进行confirm
        if mission.aobject is None:
            speak = 'what obj_detect you want to get'
            mouth.speak_via_pub(speak)
            res = ear.get_confirm(confirm_dict, target='aobject')
            if res is None:
                res = confirm_dict['aobject']
            mission.aobject = str(res)

        if mission.room is None:
            speak = 'which room you want to me go'
            mouth.speak_via_pub(speak)
            res = ear.get_confirm(confirm_dict, target='room')
            if res is None:
                res = confirm_dict['room']
            mission.room = str(res)
        return mission_type, mission
    elif mission_type == 'one':
        # 是从 placement 得到物体,同时从可能要知道name和placement2
        if mission.aobject is None:
            speak = 'what obj_detect you want to get'
            mouth.speak_via_pub(speak)
            res = ear.get_confirm(confirm_dict, target='aobject')
            if res is None:
                res = confirm_dict['aobject']
            mission.aobject = str(res)

        if mission.get_room is None:
            speak = 'which room is that obj_detect in'
            mouth.speak_via_pub(speak)
            res = ear.get_confirm_placement('get_room')
            if res is None:
                # 这个值得注意
                res = confirm_dict['get_room']
            mission.get_room = res

        if mission_ctype == 1:
            if mission.target_person is None:
                speak = 'who do you want to give this thing to'
                mouth.speak_via_pub(speak)
                res = ear.get_confirm(confirm_dict, target='pname')
                if res is None:
                    res = confirm_dict['pname']
                mission.target_person = str(res)

                # if str(res) != 'me':
                #     # 测试是否需要重新确定一遍
                #     if mission.target_room is None:
                #         speak = 'which room you want to me go when I get the obj_detect'
                #         mouth.speak_via_pub(speak)
                #         res = ear.get_confirm_placement('room')
                #         mission.target_room = str(res)

        if mission_ctype == 2:
            if mission.target_room is None:
                speak = 'which room you want to me go when I get the obj_detect'
                mouth.speak_via_pub(speak)
                res = ear.get_confirm(confirm_dict, target='target_room')
                if res is None:
                    res = confirm_dict['target_room']
                mission.target_room = str(res)
            if mission.target_person is None:
                speak = 'who do you want to give this thing to'
                mouth.speak_via_pub(speak)
                res = ear.get_confirm(confirm_dict, target='pname')
                if res is None:
                    res = confirm_dict['pname']
                mission.target_person = str(res)
        if mission_ctype == 3:
            if mission.target_room is None:
                speak = 'which room you want to me go when I get the obj_detect'
                mouth.speak_via_pub(speak)
                res = ear.get_confirm_placement('target_room', confirm_dict)
                if res is None:
                    res = confirm_dict['target_room']
                mission.target_room = str(res)
        return mission_type, mission
    elif mission_type == 'three':
        if mission.room is None:
            speak = 'which room you want to me go'
            mouth.speak_via_pub(speak)
            res = ear.get_confirm(confirm_dict, target='room')
            if res is None:
                res = confirm_dict['room']
            mission.room = str(res)

        if mission_ctype == 1:
            if mission.talk is None :
                speak = 'what you want to tell'
                mouth.speak_via_pub(speak)
                talk = ear.get_talk_via_cloud()
                mission.talk = talk

        if mission_ctype == 2:
            speak = 'please tell me what should I ask'
            mouth.speak_via_pub(speak)
            question = ear.get_question_via_cloud()
            mission.question = question
        return mission_type, mission


def make_gpsr_pdf(job=None, question=None):
    input_file = os.path.expanduser("~/robot_ros/gpsr.pdf")
    pdf_file = os.path.expanduser("~/Desktop/gpsr.pdf")
    file_handle = fitz.open(input_file)

    test_point = None
    if job is not None:
        test_point = fitz.Point(20, 320)
    elif question is not None:
        test_point = fitz.Point(20, 520)
    page = file_handle.loadPage(0)

    if job is not None:
        page.insertText(test_point, job.print_job())
    elif question is not None:
        page.insertText(test_point, question)
    file_handle.save(pdf_file)

    
if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=False)
    rospy.Rate(10)

    listen_action_topic = rospy.get_param("~listen_action_topic")
    speak_pub_topic = rospy.get_param("~speak_pub_topic")
    play_command = rospy.get_param("~play_command")
    speed_play = rospy.get_param("~speed_play")
    test_type = rospy.get_param("~test_type")

    mouth = Mouth(speak_pub_topic, play_command, speed_play)

    my_aiml = aiml.Kernel()

    if os.path.isfile(os.path.expanduser("~/robot_ros/src/robot/AIML/face_datasets/gpsr_brn.brn")):
        my_aiml.bootstrap(brainFile=os.path.expanduser("~/robot_ros/src/robot/AIML/face_datasets/gpsr_brn.brn"))
    else:
        my_aiml.bootstrap(learnFiles=os.path.expanduser("~/robot_ros/src/robot/AIML/face_datasets/gpsr-startup.xml"), commands="load aiml b")
        my_aiml.saveBrain(os.path.expanduser("~/robot_ros/src/robot/AIML/face_datasets/gpsr_brn.brn"))

    ear = Ear(listen_action_topic)
    rospy.on_shutdown(sigintHandle)
    # test speak
    if test_type == "speak":
        mouth.speak_via_pub('请再说一遍你说的是谁')
        mouth.speak_via_pub('请再说一遍你想拿什么东西')
    # test cloud
    elif test_type == "cloud":
        print('start cloud')
        question = ear.get_question_via_cloud()
        rospy.loginfo(question)
    # test local
    elif test_type == "local_gpsr":
        mission_type, mission_ctype, mission, confirm_dict = ear.get_res()
        mission_type, mission = confirm_job(mission_type, mission_ctype, mission, confirm_dict)
        
        rospy.loginfo('get message')

        make_gpsr_pdf(mission)
        # rospy.loginfo(mission.room)
        rospy.loginfo(my_aiml.respond(mission.print_job()))
    elif test_type == "local_home":
        people_name, obj_name = ear.get_name_and_object()

        rospy.loginfo('people_name is {}, obj_name is {}'.format(people_name, obj_name))
    elif test_type == "awake":
        mission_type, mission_ctype, mission, confirm_dict = ear.start_wake()
        # confirm_job(mission_type, mission_ctype, mission, confirm_dict)

        rospy.loginfo('get command')
        mission.print_job()
    elif test_type == "follow":
        res = ear.get_stop_via_awake()
        rospy.loginfo('stop follow')
    elif test_type == "confirm_placement":
        confirm_content = ear.get_confirm_placement()
    elif test_type == "confirm_room":
        confirm_content = ear.get_confirm(target='room')
        rospy.loginfo('confirm content is {}'.format(confirm_content))
    elif test_type == "confirm_me":
        confirm_content = ear.get_confirm(target='me')
    elif test_type == "confirm_name":
        confirm_content = ear.get_confirm(target='name')
        rospy.loginfo('name is {}'.format(confirm_content))

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")