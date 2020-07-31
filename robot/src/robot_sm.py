#!/usr/bin/python

import rospy
import smach

import smach_ros
from body import Robot
from sm import FindPeople, InitRobot, LookingGoods, MeetPeople, BackInit, WaitDoorOpen


def main():
    rospy.init_node('robot_state_machine')

    robot = Robot()

    sm = smach.StateMachine(outcomes=['exit'])

    rospy.loginfo('start initialize smach')
    with sm:
        smach.StateMachine.add('waitOpen', WaitDoorOpen(robot),
                               transitions={'waiting':'waitOpen',
                                            'door_opened':'initRobot'})
        smach.StateMachine.add('initRobot', InitRobot(robot),
                               transitions={'exit_state':'FindPeople'})
        smach.StateMachine.add('FindPeople', FindPeople(robot),
                               transitions={'finished':'LookingGoods',
                                            'fail':'FindPeople'})
        smach.StateMachine.add('LookingGoods', LookingGoods(robot),
                               transitions={'finished':'MeetPeople',
                                            'fail':'exit'})
        smach.StateMachine.add('MeetPeople', MeetPeople(robot),
                               transitions={'finished':'BackInit',
                                            'fail':'exit'})
        smach.StateMachine.add('BackInit', BackInit(robot),
                               transitions={'finished':'exit'})

    sis = smach_ros.IntrospectionServer('robot_state_machine', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    while not rospy.is_shutdown():
        rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()