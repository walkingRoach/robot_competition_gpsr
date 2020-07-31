# -*- coding: utf-8 -*-

from communication import Ear, Mouth
from grasp import Arm
from walk import Leg
from perception import Eye
from memory import Memory, TargetPeople
from control import Control
# from follower import ObjTracker
from config.config import Config
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from random import choice
import os
import cv2
import signal
import fitz
import sys
reload(sys)
sys.setdefaultencoding('utf-8')


class InputTimeoutError(Exception):
    pass


def interrupted(signum, frame):
    raise InputTimeoutError


class Robot:
    def __init__(self):
        self.config = Config

        self.ear = Ear(self.config.listen_action_topic)
        self.mouth = Mouth(self.config)
        self.leg = Leg()
        self.memory = Memory()
        self.arm = Arm(self.config)
        self.eye = Eye(self.config.yolo_action_topic, self.config.facenet_model, None, self.config.enable_facenet)
        # self.tracker = ObjTracker(self.eye)

        if self.config.enable_control:
            self.control = Control(self.config.light_port_name, self.config.light_baud, self.config.enable_light)

        self.current_pose = None
        self.could_grasp = self.config.enable_arm
        self.nav_dict = self.config.gpsr.nav_dict

    def response_people(self, content):
        # rospy.loginfo('获取内容同时回答')
        # raw_text = self.ear.get_res()
        job = None
        res = None
        while not res:
            # raw_text, msg = self.ear.get_res()
            res = self.mouth.respond(content)
            if not res:
                rospy.loginfo('我没听清楚')
                self.mouth.speak_via_pub('这个问题我无法解决，请重新问一个问题')
                continue
            else:
                rospy.loginfo(res)

            if res.startswith('m'):
                fina_res = res[1:-8]
                rospy.loginfo(fina_res)
                self.speak(fina_res)
                job = self.ear.parser_raw(fina_res)
                return job

    # def take_work(self, command):

    def nav_by_place_name(self, name):
        pose = self.get_place_dict(name)
        try_num = 0
        while not self.leg.move(pose, self.config.move_time_limit) and try_num < 2:
            try_num += 1
        if try_num == 2:
            return False
        else:
            return True

    def get_place_direction(self, place):
        return self.config.direction_dict[place]

    def search_by_place_name(self, name, obj_name):
        placement_list = self.config.room_dict[name]
        rospy.loginfo("I need to search {} and find {}".format(placement_list, obj_name))

        for placement in placement_list:
            if self.nav_by_place_name(placement):
                direction = self.config.direction_dict[placement]
                obj_pose = self.find_obj_poses(obj_name, direction)
                if obj_pose is not None:
                    for pose in obj_pose:
                        self.nav_by_poes(pose)

                        if self.close_obj_via_follow(obj_name):
                            self.speak('find {}'.format(obj_name))
                        else:
                            self.speak('find {}'.format(obj_name))
                    return True

        return False

    def nav_by_poes(self, pose):
        try_num = 0
        while not self.leg.move(pose, self.config.move_time_limit) and try_num < 2:
            try_num += 1
        if try_num == 2:
            return False
        else:
            return True

    def get_place_dict(self, name):
        pose = self.config.nav_dict[name]
        return pose

    def speak(self, msg):
        self.mouth.speak_via_pub(msg)
        # rospy.sleep(4)
        return True

    def remove_no_match_pose(self, pose, place):
        place_range = self.config.range_dict[place]

        if (pose.position.x > place_range[0]) and (pose.position.x < place_range[1]) and (pose.position.y > place_range[2]) and (pose.position.y < place_range[3]):
            rospy.loginfo('this pose is match this place')
            return False
        else:
            rospy.loginfo('this pose is not match this place')
            return True

    def find_people_poses(self, direction, place):
        people_boxes, people_img, people_name = self.eye.detect_obj_message('people')
        if people_boxes:
            rospy.loginfo('find %s people', len(people_boxes))
            people_poses = self.eye.get_poses(people_boxes, direction, distance=0.3)
            for pose in people_poses:
                if self.remove_no_match_pose(pose, place):
                    people_poses.remove(pose)
                else:
                    pose.orientation = self.change_quaternition(place)
            return people_poses
        else:
            rospy.loginfo('fail get people poses from current pose!')
            return None

    def find_people_pose(self, direction, place='find_people', rule='random', save=True):
        '''
        find a people pose depend on one match rules
        :return: pose
        '''
        people_boxes, people_img, people_name = self.eye.detect_obj_message('people')
        if people_boxes:
            rospy.loginfo('find %s people', len(people_boxes))
            if save:
                for box in people_boxes:
                    people_img = cv2.rectangle(people_img, (box[0], box[2]), (box[1], box[3]), (255, 0, 0), 2, 2)
                cv2.imwrite(self.config.people_img_save, people_img)

            people_poses = self.eye.get_poses(people_boxes, direction, distance=0.2)
            for pose in people_poses:
                if self.remove_no_match_pose(pose, place):
                    people_poses.remove()
            if rule == 'random':
                people_pose = choice(people_poses)
                people_pose.orientation = self.change_quaternition(place)
                return people_pose
        else:
            rospy.loginfo('fail get people from current pose')
            return None

    def find_obj_poses(self, obj, direction, save=True):
        obj_boxes, obj_img, obj_name = self.eye.detect_obj_message(obj)
        # people_img = self.eye.put_text_rec(obj_boxes, obj_img)
        # save_path = self.config.people_img_save + datetime.datetime.now().strftime("%D:%H:%M") + '.jpg'
        # cv2.imwrite('/home/ubuntu/Desktop/1.jpg', people_img)
        if obj_boxes:
            if save:
                for obj_box in obj_boxes:
                    obj_img = cv2.rectangle(obj_img, (obj_box[0], obj_box[2]), (obj_box[1], obj_box[3]), (255, 255, 0), 2, 2)
                cv2.imwrite(self.config.obj_img_save, obj_img)
            # rospy.loginfo('find %s'%obj)
            obj_pose = self.eye.get_poses(obj_boxes, direction, distance=1.0)
            for pose in obj_pose:
                pose.orientation = self.change_quaternition('take_goods')
            return obj_pose
        else:
            rospy.loginfo('fail get obj')
            return None

    def recognize(self):
        face_boxes, face_img = self.eye.detect_obj_message('face')

        if not face_boxes:
            for face_box in face_boxes:
                if face_box.Class == 'stranger':
                    rospy.loginfo('我不认识你')
                    return False
                else:
                    rospy.loginfo('你好%s'%face_box.Class)
                    # self.memory.add_face(face_img)
                    return True
        else:
            rospy.loginfo('fail get face_boxes')
            return False

    def ask_for_person(self, choose):
        # choose 1 ask time or weather,
        # choose 2 ask command
        if choose == 1:
            rospy.loginfo('what do you want to ask me, you can ask me time or weather')
            if self.ask_to_person():
                raw_text, res = self.ear.get_res()
            self.speak(res)
            return None
        elif choose == 2:
            rospy.loginfo('help me take something')
            job = self.ear.get_job()
            self.memory.add_job(job)
            return job
        else:
            return None

    def speak_wav(self, content):
        '''
        using wav file speak
        :param content: 1 hello, 2 again, 3 tip, 4 speak,
                        5 cant solve, 6 bye, 7 put hand,
                        8 put head, 9 Want to do, 10 Follow,
                        11 hello name
        :return:
        '''
        wav_file = self.config.gpsr.get_wav(content)
        if wav_file is not None:
            self.mouth.speak_via_wav(wav_file)

    # 以后在写，先弄懂点云数据。
    def follow(self, follow_obj='people'):
        self.tracker.init_track(follow_obj)
        tracker_stop = False
        twist_zore = 0
        while not tracker_stop and twist_zore < 10:
            try:
                twist = rospy.wait_for_message('/cmd_vel', Twist, 1)
                if twist.linear.x == 0:
                    rospy.loginfo('twist_zore add one')
                    twist_zore += 1
                else:
                    twist_zore = 0
            except:
                twist_zore += 1
                stop_msg = Twist()
                self.leg.cmd_vel_pub.publish(stop_msg)
            if not self.tracker.is_tracking:
                tracker_stop = True
        rospy.loginfo('follow end')

    def follow_with_voice(self, follow_obj='people'):
        if self.tracker.init_track(follow_obj):
            rospy.loginfo('success track')
        else:
            return False
        self.ear.get_stop_via_awake()
        return True

    def add_face2db(self, people_name, people_img):
        self.eye.recognition.add_db(people_name, people_img)

    def close_obj(self, close_obj):
        stop = False
        while not stop:
            obj_boxes, obj_img, obj_name = self.eye.detect_obj(close_obj)
            if obj_boxes is None:
                rospy.loginfo('fail get obj_boxes')
                return False

            goal_x, goal_z = self.eye.get_cmd(obj_boxes, obj_img)
            if goal_z == 0:
                rospy.loginfo('stop')
                stop = True

            rospy.loginfo('goal_x : {}, goal_z : {}'.format(goal_x, goal_z))
            # goal_z = 0.05
            # goal_x = -0.15
            self.leg.approach_obj(0.0, goal_z)
            rospy.loginfo('success get obj')
        self.leg.approach_obj(0.20, 0.0)
        return True

    def close_obj_via_follow(self, obj, stop_twist_x=0.015):
        # 开始跟踪
        if self.tracker.init_track(obj):
            # 进行强制定时办法
            # rospy.sleep(5)
            # 订阅速度节点
            stop_close = False
            stop_count = 0
            while not stop_close:
                twist = rospy.wait_for_message('/cmd_vel', Twist)
                if twist.linear.x < stop_twist_x and twist.angular.z < 0.01:
                    stop_count += 1
                else:
                    stop_count = 0

                if stop_count > 10:
                    rospy.loginfo('stop close')
                    self.ear.stop_follow_pub.publish(False)
                    return True
        else:
            return False

    def change_quaternition(self, address):
        current_quaternition = self.get_place_dict(address).orientation
        rospy.loginfo('current_quaternition：%s'%current_quaternition)
        return current_quaternition

    def set_clock(self, hour, minute):
        msg = String()
        msg.data = '{},{}'.format(str(hour), str(minute))
        self.clock_pub.publish(msg)

    def get_current_pose(self):
        pose = self.leg.get_pose()
        return pose

    def confirm_job(self, mission_type, mission_ctype, mission, confirm_dict):
        speak = None
        if mission_type == 'two':
            # 这个最简单,使用local进行confirm
            if mission.aobject is None:
                if not self.config.use_chinese_confirm:
                    speak = 'what obj_detect you want to get'
                else:
                    speak = '你想要拿什么'
                self.mouth.speak_via_pub(speak)
                res = self.ear.get_confirm(confirm_dict, target='aobject')
                if res is None:
                    res = confirm_dict['aobject']
                res = self.config.obj_to_true[res]
                mission.aobject = str(res)

            if mission.room is None:
                if not self.config.use_chinese_confirm:
                    speak = 'which room you want to me go'
                else:
                    speak = '你想要我去哪个房间'
                self.mouth.speak_via_pub(speak)
                res = self.ear.get_confirm(confirm_dict, target='room')
                if res is None:
                    res = confirm_dict['room']
                mission.room = str(res)
            return mission_type, mission
        elif mission_type == 'one':
            # 是从 placement 得到物体,同时从可能要知道name和placement2
            if mission.aobject is None:
                if not self.config.use_chinese_confirm:
                    speak = 'what obj_detect you want to get'
                else:
                    speak = '你想我拿什么'
                self.mouth.speak_via_pub(speak)
                res = self.ear.get_confirm(confirm_dict, target='aobject')
                if res is None:
                    res = confirm_dict['aobject']
                res = self.config.obj_to_true[res]
                mission.aobject = str(res)

            if mission.get_room is None:
                if not self.config.use_chinese_confirm:
                    speak = 'which room is that obj_detect in'
                else:
                    speak = '你想我去哪个位置拿物品'
                self.mouth.speak_via_pub(speak)
                res = self.ear.get_confirm_placement('get_room')
                if res is None:
                    # 这个值得注意
                    res = str(confirm_dict['get_room']).strip().replace(' ', '_')
                mission.get_room = res

            if mission_ctype == 1:
                if mission.target_person is None:
                    if not self.config.use_chinese_confirm:
                        speak = 'who do you want to give this thing to'
                    else:
                        speak = '我应该把物体给谁'
                    self.mouth.speak_via_pub(speak)
                    res = self.ear.get_confirm(confirm_dict, target='pname')
                    if res is None:
                        res = confirm_dict['pname']
                    mission.target_person = str(res)

                    if str(res) != 'me':
                        # 测试是否需要重新确定一遍
                        if mission.target_room is None:
                            if not self.config.use_chinese_confirm:
                                speak = 'which room you want to me go when I get the obj_detect'
                            else:
                                speak = '你想要我将物体运到哪个位置'
                            self.mouth.speak_via_pub(speak)
                            res = self.ear.get_confirm_placement('room')
                            mission.target_room = str(res)

            if mission_ctype == 2:
                if mission.target_room is None:
                    if not self.config.use_chinese_confirm:
                        speak = 'which room you want to me go when I get the obj_detect'
                    else:
                        speak = '拿到物体后我应该去哪个房间'
                    self.mouth.speak_via_pub(speak)
                    res = self.ear.get_confirm(confirm_dict, target='target_room')
                    if res is None:
                        res = str(confirm_dict['target_room']).strip().replace(' ', '_')
                    mission.target_room = str(res)
                if mission.target_person is None:
                    if not self.config.use_chinese_confirm:
                        speak = 'who do you want to give this thing to'
                    else:
                        speak = '我应该将物体给谁'
                    self.mouth.speak_via_pub(speak)
                    res = self.ear.get_confirm(confirm_dict, target='pname')
                    if res is None:
                        res = confirm_dict['pname']
                    mission.target_person = str(res)
            if mission_ctype == 3:
                if mission.target_room is None:
                    if not self.config.use_chinese_confirm:
                        speak = 'which room you want to me go when I get the obj_detect'
                    else:
                        speak = '我应该将物体拿到什么位置'
                    self.mouth.speak_via_pub(speak)
                    res = self.ear.get_confirm_placement('target_room', confirm_dict)
                    if res is None:
                        res = str(confirm_dict['target_room']).strip().replace(' ', '_')
                    mission.target_room = str(res)
            return mission_type, mission
        elif mission_type == 'three':
            if mission.room is None:
                if not self.config.use_chinese_confirm:
                    speak = 'which room you want to me go'
                else:
                    speak = '我应该去哪个房间'
                self.mouth.speak_via_pub(speak)
                res = self.ear.get_confirm(confirm_dict, target='room')
                if res is None:
                    res = confirm_dict['room']
                mission.room = str(res)

            if mission_ctype == 1:
                if mission.talk is None:
                    speak = 'what you want to tell'
                    self.mouth.speak_via_pub(speak)
                    talk = self.ear.get_talk_via_cloud()
                    mission.talk = talk

            if mission_ctype == 2:
                pass
                # if not self.config.use_chinese_confirm:
                #     speak = 'please tell me what should I ask'
                # else:
                #     speak = '你想让我问什么'
                # self.mouth.speak_via_pub(speak)
                # question = self.ear.get_question_via_cloud()
                # mission.question = question
            return mission_type, mission

    def add_people(self):
        '''
        aim to add people message to Memory class
        '''

        self.mouth.speak_via_pub('你好请低下头看着我')

        i, try_times = 0, 3
        detect_stop = False
        face_feature, face_img = None, None
        while i < try_times and not detect_stop:
            signal.signal(signal.SIGALRM, interrupted)
            signal.alarm(5)

            try:
                face_feature, face_img = self.eye.detect_face()
                detect_stop = True
            except InputTimeoutError:
                rospy.loginfo('timeout')
                i += 1
                if i == 1:
                    self.leg.approach_obj(0.0, -0.1)
                elif i == 2:
                    self.leg.approach_obj(0.0, 0.2)
                else:
                    self.leg.approach_obj(0.0, -0.1)
                self.mouth.speak_via_pub('请你再低下头好好看看我')
                # self.leg.approach_obj(0.0, 0.1)
            signal.alarm(0)

        self.mouth.speak_via_pub('请告诉我你的姓名以及想要的物品')

        # 得到信息的内容
        people_name, obj_name = self.ear.get_name_and_object()

        face_path = os.path.expanduser("~/robot_ros/src/robot/image/people/") + str(people_name) + '_' + str(obj_name) + '.jpg'
        cv2.imwrite(face_path, face_img)

        people = TargetPeople(people_name, obj_name, face_feature, face_path)

        # self.memory.add_people(people)
        return people

    def remember_people(self):
        self.mouth.speak_via_pub('你好请低下头看着我')

        i, try_times = 0, 4
        detect_stop = False
        face_feature, face_img = None, None
        while i < try_times and not detect_stop:
            signal.signal(signal.SIGALRM, interrupted)
            signal.alarm(5)

            try:
                face_feature, face_img = self.eye.detect_face()
                detect_stop = True
            except InputTimeoutError:
                rospy.loginfo('timeout')
                i += 1
                self.mouth.speak_via_pub('请你再低下头好好看看我')

            signal.alarm(0)

        face_path = os.path.expanduser("~/robot_ros/src/robot/image/people/") + 'me' + '_' + str(i) + '.jpg'
        cv2.imwrite(face_path, face_img)

        return face_feature, face_img

    def get_name(self):
        name = self.ear.get_confirm(target='name')
        if name is None:
            self.speak('对不起我不能识别你的名字')
        return name

    def identify_people(self):
        '''
        depond on myself to identify from detecting face to get his name need test
        :return: name
        '''
        self.mouth.speak_via_pub('你好请低下头看着我')

        i, try_times = 0, 4
        detect_stop = False
        face_feature, face_img = None, None
        while i < try_times and not detect_stop:
            signal.signal(signal.SIGALRM, interrupted)
            signal.alarm(5)

            try:
                face_feature, face_img = self.eye.detect_face()
                detect_stop = True
            except InputTimeoutError:
                rospy.loginfo('timeout')
                i += 1
                if i == 1:
                    self.leg.approach_obj(0.0, -0.1)
                elif i == 2:
                    self.leg.approach_obj(0.0, 0.2)
                else:
                    self.leg.approach_obj(0.0, -0.1)
                self.mouth.speak_via_pub('请你再低下头好好看看我')

            signal.alarm(0)
        name = None
        best = 10.0
        '''
        for db in self.memory.people_with_job_list:
            dist = np.sqrt(np.sum(np.square(np.subtract(face_feature, db.peopleFace))))
            if dist < best:
                best = dist
                name = db.peopleName
        '''
        for db in self.memory.people_with_job_list:
            if db.peopleFace is not None:
                dist = (face_feature - db.peopleFace).norm().item()
                rospy.loginfo('dist is {}'.format(dist))
                if dist < best:
                    best = dist
                    name = db.peopleName

        if best < 0.8:
            return name
        else:
            return None

    def talk_to_person(self, msg):
        rospy.loginfo("get msg is {}".format(msg))
        res = self.mouth.respond(msg)

        rospy.loginfo(res)

        self.mouth.speak_via_pub(res)

    def ask_to_person(self, question=None):
        # 之后得到问题
        if question is None:
            self.mouth.speak_via_pub('你有什么问题想要问我吗')
            question = self.ear.get_question_via_cloud()

            if question is not None:
                res = self.mouth.respond(question)

                self.mouth.speak_via_pub(res)
                self.make_gpsr_pdf(question=str(question))
                return True
            else:
                self.mouth.speak_via_pub('对不起我不能听懂你提的问题')
                return False
        else:
            res = self.mouth.respond(question)
            self.mouth.speak_via_pub(res)

    def make_pdf(self):
        pdf_file = os.path.expanduser("~/Desktop/people.pdf")

        people_path = os.path.expanduser('~/robot_ros/src/robot/image/people')

        image_list = os.listdir(people_path)
        input_file = os.path.expanduser("~/robot_ros/test.pdf")
        file_handle = fitz.open(input_file)

        for image_name in image_list:

            barcode_file = os.path.join(people_path, image_name)

            # define the position (upper-right corner)
            image_rectangle = fitz.Rect(20, 20, 300, 300)

            test_point = fitz.Point(20, 320)
            # retrieve the first page of the PDF

            page = file_handle.newPage()

            # add the image
            page.insertImage(image_rectangle, filename=barcode_file)

            page.insertText(test_point, image_name.split('.')[0])

        file_handle.save(pdf_file)

    def make_gpsr_pdf(self, job=None, question=None):
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

    def clean_img(self):
        people_path = os.path.expanduser('~/robot_ros/src/robot/image/people')

        file_list = os.listdir(people_path)

        for file_name in file_list:
            os.remove(os.path.join(people_path, file_name))

    def get_height_via_name(self, place):
        return self.config.grasp_height[place]

    # def get_pose_via_scan(self):

