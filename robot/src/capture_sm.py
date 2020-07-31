#!/usr/bin/python

import rospy
import smach

import smach_ros

from body import Robot
from sm import InitRobot, LookingGoods, MeetPeople, BackInit


def main():
    rospy.init_node('robot_state_machine')

    robot = Robot()

    sm = smach.StateMachine(outcomes=['exit'])

    rospy.loginfo('start initialize smach')
    with sm:
        smach.StateMachine.add('init', InitRobot(robot),
                               transitions={'exit_state':'MeetPeople'})
        smach.StateMachine.add('MeetPeople', MeetPeople(robot),
                               transitions={'finished':'LookingGoods',
                                            'fail':'BackInit',
                                            'bye':'BackInit'})
        smach.StateMachine.add('LookingGoods', LookingGoods(robot),
                               transitions={'finished':'BackInit',
                                            'fail':'BackInit'})
        smach.StateMachine.add('BackInit', BackInit(robot),
                               transitions={'finished':'exit'})
        # smach.StateMachine.add('LookingGoods', LookingGoods(robot),
        #                        transitions={'finished':'FindPeople',
        #                                     'fail':'LookingGoods'})

    sis = smach_ros.IntrospectionServer('robot_state_machine', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    while not rospy.is_shutdown():
        rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()