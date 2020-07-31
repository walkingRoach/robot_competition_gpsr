import os
from geometry_msgs.msg import Pose, Point, Quaternion


class GPSR:
    def __init__(self, wav_root):
        self.wav_base_path = wav_root

        self.nav_dict = dict()
        self.room_dict = dict()
        self.room_range_dict = dict()
        self.grasp_height = dict()
        self.obj_dict = dict()
        self.wav_dict = dict()

        self.init_param()

    def init_param(self):
        self.nav_dict['init_pose'] = (Pose(Point(7.47, -6.3, 0.0),
                                      Quaternion(0.000, 0.000, 0.70, 0.71)), 'left')
        # self.nav_dict['find_people'] = Pose(Point(1.02, 0.4, 0.0),
#                               Quaternion(0.000, 0.000, -0.73, 0.68))
        self.nav_dict['find_people'] = (Pose(Point(7.47, -8.0, 0.0),
                                        Quaternion(0.000, 0.000, 0.70, 0.71)), 'left')
        self.nav_dict['take_goods'] = (Pose(Point(3.65, -5.67, 0.0),
                                       Quaternion(0.000, 0.000, 0.0, 0.99)), 'left')
        self.nav_dict['back_pose'] = (Pose(Point(0.0, 0.0, 0.0),
                              Quaternion(0.000, 0.000, 0.99, 0.0)), 'down')
        self.nav_dict['back_find_people'] = (Pose(Point(6.86, -2.86, 0.0),
                                             Quaternion(0.000, 0.000, -0.73, 0.69)), 'right')
        self.nav_dict['livingroom'] = (Pose(Point(6.98, -4.0, 0.0),
                                       Quaternion(0.000, 0.000, -0.73, 0.68)), 'right')
        self.nav_dict['kitchen'] = (Pose(Point(3.33, -5.68, 0.0),
                                    Quaternion(0.000, 0.000, -0.73, 0.68)), 'right')
        self.nav_dict['bedroom'] = (Pose(Point(6.7, -2.25, 0.0),
                                    Quaternion(0.000, 0.000, 0.70, 0.71)), 'left')
        self.nav_dict['hallway'] = (Pose(Point(4.51, -3.35, 0.0),
                                    Quaternion(0.000, 0.000, 0.70, 0.71)), 'left')
        self.nav_dict['kitchen_table'] = (Pose(Point(2.67, -5.6, 0.0),
                                          Quaternion(0.000, 0.000, 0.01, 0.99)), 'right')  # 左边
        self.nav_dict['kitchen_counter'] = (Pose(Point(4.6, -9.65, 0.0),
                                             Quaternion(0.000, 0.000, 0.01, 0.99)), 'up')   # 最右边
        self.nav_dict['right_bedside_table'] = (Pose(Point(7.26, -0.8, 0.0),
                                                Quaternion(0.000, 0.000, 0.01, 0.99)), 'up')
        self.nav_dict['shelf'] = (Pose(Point(6.3, -0.1, 0.0),
                                  Quaternion(0.000, 0.000, 0.99, 0.1)), 'up')   # -0.74, 0.66, 上方那个架子
        self.nav_dict['bar'] = (Pose(Point(7.71, -2.78, 0.0),
                                Quaternion(0.000, 0.000, 0.0, 0.99)), 'up')  # 下方的哪个桌子
        self.nav_dict['desk'] = (Pose(Point(1.14, -1.64, 0.0),
                                 Quaternion(0.000, 0.000, 0.01, 0.99)), 'up')   # 上方

        self.room_dict['kitchen'] = ('kitchen_table', 'kitchen_counter')  # kitchen_counter目前被放弃
        self.room_dict['livingroom'] = ('bar')
        self.room_dict['hallway'] = ('desk')
        self.room_dict['bedroom'] = ('shelf', 'right_bedside_table')

        self.room_range_dict['hallway'] = (0.6, 4.49, -5.13, 2.07)
        self.room_range_dict['livingself.room'] = (6.23, 9.82, -9.87, -2.4)
        self.room_range_dict['kitchen'] = (1.95, 5.23, -10.2, -4.59)
        self.room_range_dict['bedself.room'] = (4.89, 8.24, -2.68, 2.69)
        self.room_range_dict['find_people'] = (6.23, 9.82, -9.75, -2.2)
        self.room_range_dict['back_find_people'] = (6.23, 9.82, -9.75, -2.2)

        self.grasp_height['kitchen_table'] = 70
        self.grasp_height['kitchen_counter'] = 150
        self.grasp_height['bar'] = 70
        self.grasp_height['shelf'] = 150
        self.grasp_height['right_bedside_table'] = 70
        self.grasp_height['desk'] = 70

        self.obj_dict['bathCream'] = 'bathCream'
        self.obj_dict['milkTea'] = 'milkTea'
        self.obj_dict['napkin'] = 'napkin'
        self.obj_dict['oreo'] = 'oreo'
        self.obj_dict['pepsi'] = 'pepsi'
        self.obj_dict['chips'] = 'chips'
        self.obj_dict['green'] = 'greenTea'
        self.obj_dict['herbal'] = 'herbalTea'
        self.obj_dict['porridge'] = 'porridge'
        self.obj_dict['biscuists'] = 'biscuists'

        self.wav_dict['self_intro'] = self.wav_base_path + "self_intro.wav"
        self.wav_dict['hello'] = self.wav_base_path + "hello.wav"
        self.wav_dict['cant_solve'] = self.wav_base_path + "cant_solve.wav"
        self.wav_dict['again'] = self.wav_base_path + "again.wav"
        self.wav_dict['speak'] = self.wav_base_path + "speak.wav"
        self.wav_dict['tip'] = self.wav_base_path + "tip.wav"
        self.wav_dict['bye'] = self.wav_base_path + "bye.wav"
        self.wav_dict['put_hand'] = self.wav_base_path + "put_hand.wav"
        self.wav_dict['follow'] = self.wav_base_path + "follow.wav"
        self.wav_dict['hello_ouquanlin'] = self.wav_base_path + "hello_ou.wav"

    def get_pose(self, pose_name):
        return self.nav_dict[pose_name][0]

    def get_dict(self, pose_name):
        return self.nav_dict[pose_name][1]

    def get_pose_and_dict(self, pose_name):
        return self.nav_dict[pose_name][0], self.nav_dict[pose_name][1]

    def get_loc_by_room(self, room_name):
        return self.room_dict[room_name]

    def get_room_range(self, room_name):
        return self.room_range_dict[room_name]

    def get_wav(self, name):
        return self.wav_dict[name]

    def get_obj(self, obj_name):
        return self.obj_dict[obj_name]
