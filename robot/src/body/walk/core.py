#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from actionlib.msg import *
from actionlib_msgs.msg import GoalStatus
import time
import tf

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from math import radians, copysign, sqrt, pow
from transform_utils import *


class Leg:
    def __init__(self):
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.move_cmd = Twist()
        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform('/odom', '/base_link', rospy.Time(), rospy.Duration(1.0))
        self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(1.0))
        # self.lock = thread.allocate_lock()

    def move(self, pose, limit_time=30, frame_id='map'):
        goal = MoveBaseGoal()
        goal.target_pose.pose = pose
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        start = time.time()
        rospy.loginfo('send goal')

        self.move_base.send_goal(goal)

        self.move_base.wait_for_server(rospy.Duration(20))
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(limit_time))

        if not finished_within_time:
            rospy.loginfo('fail reach loc')
            self.move_base.cancel_goal()

            return False
        else:
            rospy.loginfo('arrived at pose in %s', time.time() - start)
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo('goal reach')
                rospy.loginfo('print move state {}'.format(state))

                return True
            else:
                rospy.loginfo('goal fail {}'.format(state))
                return False

    def approach_obj(self, goal_x=0.1, goal_z=0.2):
        rospy.loginfo('goal_x : {}, goal_z : {}'.format(goal_x, goal_z))
        # 使用准确的移动方式进行控制
        rate = 20

        loopRate = rospy.Rate(rate)
        # 设定标准的移动速度和角速度
        if goal_x > 0:
            linear_speed = 0.2
        else:
            linear_speed = -0.2

        if goal_z > 0:
            angular_speed = 0.5
        else:
            angular_speed = -0.5

        angular_tolerance = radians(1.5)

        move_cmd = Twist()

        # 获得当前的位置信息
        position, rotation = self.get_odom()
        rospy.loginfo('current pose is {}, {}'.format(position, rotation))

        x_start = position.x
        y_start = position.y

        move_cmd.angular.z = angular_speed

        last_angle = rotation

        turn_angle = 0

        while abs(turn_angle + angular_tolerance) < abs(goal_z):
            self.cmd_vel_pub.publish(move_cmd)
            loopRate.sleep()

            position, rotation = self.get_odom()

            delta_angle = normalize_angle(rotation - last_angle)

            turn_angle += delta_angle
            last_angle = rotation

        move_cmd = Twist()
        self.cmd_vel_pub.publish(move_cmd)
        rospy.sleep(1.0)

        distance = 0
        rospy.loginfo('start change x')
        move_cmd.linear.x = linear_speed

        while distance < abs(goal_x):
            self.cmd_vel_pub.publish(move_cmd)

            loopRate.sleep()

            position, rotation = self.get_odom()

            distance = sqrt(pow(abs(position.x - x_start), 2) +
                            pow(abs(position.y - y_start), 2))

        move_cmd = Twist()
        self.cmd_vel_pub.publish(move_cmd)
        rospy.sleep(1.0)

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("tf exception")
            return False

        return Point(*trans), quat_to_angle(Quaternion(*rot))

    def get_pose(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("tf exception")
            return False

        position = Point(*trans)
        orientation = Quaternion(*rot)

        pose = Pose(position, orientation)

        rospy.loginfo("current pose is %s", pose)

        return pose

        # return Point(*trans), Quaternion(*rot))

