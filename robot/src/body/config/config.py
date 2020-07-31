# -*- coding: utf-8 -*-
import os
from .gpsr import GPSR


class Config:
    poses_file_path = os.path.expanduser("~/home_ws/.cache/poses_20180808.xml")

    # robot 对象中 topic
    speak_pub_topic = "/xf/tts/words"
    asr_action_topic = "/xf_asr/home_recognize"
    listen_action_topic = "/xf_asr/home_recognize"
    tts_action_topic = "/xf_tts/tts_generate"
    yolo_action_topic = "/darknet_ros/check_for_objects"

    base_path = os.path.expanduser("~/robot_ros/src/robot/")
    # 常规文件地址
    wav_base_path = base_path + "wav/"
    people_img_save = os.path.expanduser("~/robot_ros/img/people_pose/people.jpg")
    obj_img_save = os.path.expanduser("~/robot_ros/img/obj_img/obj.jpg")
    aiml_file = os.path.expanduser("~/robot_ros/src/robot/AIML/face_datasets/gpsr-startup.xml")
    brn_path = os.path.expanduser("~/robot_ros/src/robot/AIML/face_datasets/gpsr_brn.brn")

    # wav part
    play_command = "aplay"
    wav_speed_up = "1.2"  # Notes: use str
    use_chinese_confirm = True  # 使用中文语音回答

    # arm part
    enable_arm = True
    arm_port_name = '/dev/arm'
    arm_baud = 115200

    # light control
    enable_control = False
    enable_light = False
    light_port_name = '/dev/control'
    light_baud = 9600

    # debug part
    debug = True
    debug_path = base_path + "debug/"
    final_debug_path = base_path + "final_debug/"

    # facenet part
    enable_facenet = True
    facenet_model = os.path.expanduser("~/robot_ros/src/robot/models/facenet/20180402-114759/20180402-114759.pb")
    database_path = os.path.expanduser("~/robot_ros/src/robot/face_datasets/")
    # enable_facenet = rospy.get
    facenet_each_person_face_num = 3

    # map related
    xmin = 0.347
    ymin = -1.43
    xmax = 9.25
    ymax = 10.9

    # TODO(lennon) is move 30 second enough, need test in final home environment
    move_time_limit = 30000
    # follow定义的参数
    follow_distance = 0.4
    follow_confidence_threshold = None

    # 参加gpsr的参数
    gpsr = GPSR(wav_base_path)

    # 人脸辨识参数
