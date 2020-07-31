#!/usr/bin/python

import rospy
import smach

import smach_ros
from body import Robot

from gpsr_sm import FindPeople, GetObj, GetJob, GoRoom, InitRobot, BackInit


def main():
    rospy.init_node('gpsr_state_machine')

    sm = smach.StateMachine(outcomes=['exit'])

    robot = Robot()
    rospy.loginfo('start initialize smach')
    # mission1 is from somewhere take something to someone
    # mission2 is from somewhere find something
    # mission3 is from somewhere find people tell something or follow
    with sm:
        smach.StateMachine.add('init', InitRobot(robot),
                               transitions={'exit_state':'GetJob',
                                            'fail':'init'})
        smach.StateMachine.add('GetJob', GetJob(robot),
                               transitions={'mission1':'GetObj',
                                            'mission2':'GoRoom',
                                            'mission3':'FindPeople'},
                               remapping={'command':'job_data'})
        smach.StateMachine.add('GetObj', GetObj(robot),
                               transitions={'finish':'BackInit'},
                               remapping={'get_obj_in':'job_data'})
        smach.StateMachine.add('GoRoom', GoRoom(robot),
                               transitions={'finish':'BackInit'},
                               remapping={'go_room_in':'job_data'})
        smach.StateMachine.add('FindPeople', FindPeople(robot),
                               transitions={'finish':'BackInit'},
                               remapping={'find_people_in':'job_data'})
        smach.StateMachine.add('BackInit', BackInit(robot),
                               transitions={'finish':'exit'})

    sis = smach_ros.IntrospectionServer('gpsr_state_machine', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    while not rospy.is_shutdown():
        rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
