#!/usr/bin/python
# -*- coding: utf-8 -*-

import aiml
import sys
import os

reload(sys)
sys.setdefaultencoding('utf-8')

if __name__ == '__main__':
    my_aiml = aiml.Kernel()

    if os.path.exists(os.path.expanduser("~/robot_ros/src/robot/AIML/face_datasets/gpsr_brn.brn")):
        my_aiml.bootstrap(brainFile=os.path.expanduser("~/robot_ros/src/robot/AIML/face_datasets/gpsr_brn.brn"))
    else:
        my_aiml.bootstrap(learnFiles=os.path.expanduser("~/robot_ros/src/robot/AIML/face_datasets/gpsr_startup.xml"), commands="GPSR")
        my_aiml.saveBrain(os.path.expanduser("~/robot_ros/src/robot/AIML/face_datasets/gpsr_brn.brn"))

    # my_aiml.respond('load aiml b')

    while True:
        ask = raw_input()
        res = my_aiml.respond(ask)
        if not res:
            print ('我没听清楚')
        else:
            print (res)

