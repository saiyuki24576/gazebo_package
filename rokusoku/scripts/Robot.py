#!/usr/bin/env python
# coding: utf-8

import rospy
import roslib
import time
import random
import time
import sys
import os
import tf
import math
import numpy as np

from std_msgs.msg import Int64
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.srv import GetJointProperties
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.srv import SetModelConfigurationRequest

"""
更新
self.powerをこちらのクラスの変数へ(使いがってが悪かったので)
pub_motor()を大幅改造 高速化
"""
"""
関数一覧
pub_motor():すべての関節を動かす すべての関節の角度(動かさなければそのままの同じ角度)が必要
set_start_self.power():ロボットの関節を初期状態へ戻す
reset_model():初期座標・初期状態へ戻す
get_pos():現在の位置を取得
get_joint_ang():現在のJointの角度を必要
get_center():重心を求める
_ft_1_1_call(msg):ftセンサーから値を読み取る
force_detection():現在の最大の力を取得 また、すぐ接触情報が失われるため、どうするか
"""
class SixlegController:
    #コンストラクタ
    def __init__(self, robotID=1, posofsetx=0, posofsety=0):
        #print robotID, "init"
        self.ID = str(robotID)
        self._link_num = 18
        self._joint_num = 18
        self.power = [0] * self._joint_num
        self.now_ang = [0] * self._joint_num
        #モーターが回転し終わったあとの遅延時間
        self._MOTER_DELAY = 0.05
        #ロボットの初期座標を補正
        self.offset_x = posofsetx
        self.offset_y = posofsety
        #モデルの変数の設定
        #MB=massBase MM=massSum
        self.center_x = 0
        self.center_y = 0
        self.center_z = 0
        #リンクの位置
        self.link_pos = [[0 for i in range(3)] for j in range(self._link_num)]
        #ロボットのベースの位置 リンクの位置と同じだがこれだけ特別
        self.base_x = 0
        self.base_y = 0
        self.base_z = 0
        #ロボットのベースの傾き
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.model_name = "rokusoku" + self.ID
        #Gazeboのシミュレータの速度
        self.GAZEBO_SIMULATE_TIME = 2.0
        self.T = self.GAZEBO_SIMULATE_TIME
        #link_nameの順番:base,1-1,1-2,1-3,2-1,・・・・とする
        self.link_name = [str(self.model_name)+"::base",str(self.model_name)+"::link1_1", str(self.model_name)+"::link1_2", str(self.model_name)+"::link1_3", str(self.model_name)+"::link2_1", str(self.model_name)+"::link2_2", str(self.model_name)+"::link2_3", str(self.model_name)+"::link3_1", str(self.model_name)+"::link3_2", str(self.model_name)+"::link3_3", str(self.model_name)+"::link4_1", str(self.model_name)+"::link4_2",  str(self.model_name)+"::link4_3", str(self.model_name)+"::link5_1", str(self.model_name)+"::link5_2", str(self.model_name)+"::link5_3", str(self.model_name)+"::link6_1", str(self.model_name)+"::link6_2", str(self.model_name)+"::link6_3"]
        self.link_name_all = [str(self.model_name)+"::base",str(self.model_name)+"::link1_1", str(self.model_name)+"::link1_2", str(self.model_name)+"::link1_3", str(self.model_name)+"::link1_4", str(self.model_name)+"::link2_1", str(self.model_name)+"::link2_2", str(self.model_name)+"::link2_3", str(self.model_name)+"::link2_4", str(self.model_name)+"::link3_1", str(self.model_name)+"::link3_2", str(self.model_name)+"::link3_3", str(self.model_name)+"::link3_4", str(self.model_name)+"::link4_1", str(self.model_name)+"::link4_2", str(self.model_name)+"::link4_3", str(self.model_name)+"::link4_4", str(self.model_name)+"::link5_1", str(self.model_name)+"::link5_2", str(self.model_name)+"::link5_3", str(self.model_name)+"::link5_4", str(self.model_name)+"::link6_1", str(self.model_name)+"::link6_2", str(self.model_name)+"::link6_3", str(self.model_name)+"::link6_4"]
        #ros関連の変数の設定
        self.set = ModelState()
        self.link = LinkState()
        self.getlink= GetLinkState()
        self.jointcall = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.set_pub = rospy.Publisher('/gazebo/set_model_state',ModelState,queue_size = 10)
        self.posecall  = rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)
        self.pose_link = rospy.Publisher("/gazebo/set_link_state",LinkState,queue_size = 100)
        self.reset_joints =  rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)

        #pause関連の設定
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics',Empty)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
        s1 = '/rokusoku'+self.ID+'/Joint'
        s2 = '_position_controller/command'
        self.pub_motor1_1 = rospy.Publisher(s1+'1_1'+s2,Float64,queue_size = 1)
        self.pub_motor1_2 = rospy.Publisher(s1+'1_2'+s2,Float64,queue_size = 1)
        self.pub_motor1_3 = rospy.Publisher(s1+'1_3'+s2,Float64,queue_size = 1)
        self.pub_motor2_1 = rospy.Publisher(s1+'2_1'+s2,Float64,queue_size = 1)
        self.pub_motor2_2 = rospy.Publisher(s1+'2_2'+s2,Float64,queue_size = 1)
        self.pub_motor2_3 = rospy.Publisher(s1+'2_3'+s2,Float64,queue_size = 1)
        self.pub_motor3_1 = rospy.Publisher(s1+'3_1'+s2,Float64,queue_size = 1)
        self.pub_motor3_2 = rospy.Publisher(s1+'3_2'+s2,Float64,queue_size = 1)
        self.pub_motor3_3 = rospy.Publisher(s1+'3_3'+s2,Float64,queue_size = 1)
        self.pub_motor4_1 = rospy.Publisher(s1+'4_1'+s2,Float64,queue_size = 1)
        self.pub_motor4_2 = rospy.Publisher(s1+'4_2'+s2,Float64,queue_size = 1)
        self.pub_motor4_3 = rospy.Publisher(s1+'4_3'+s2,Float64,queue_size = 1)
        self.pub_motor5_1 = rospy.Publisher(s1+'5_1'+s2,Float64,queue_size = 1)
        self.pub_motor5_2 = rospy.Publisher(s1+'5_2'+s2,Float64,queue_size = 1)
        self.pub_motor5_3 = rospy.Publisher(s1+'5_3'+s2,Float64,queue_size = 1)
        self.pub_motor6_1 = rospy.Publisher(s1+'6_1'+s2,Float64,queue_size = 1)
        self.pub_motor6_2 = rospy.Publisher(s1+'6_2'+s2,Float64,queue_size = 1)
        self.pub_motor6_3 = rospy.Publisher(s1+'6_3'+s2,Float64,queue_size = 1)
        self.pub = [self.pub_motor1_1, self.pub_motor1_2, self.pub_motor1_3, self.pub_motor2_1,
                    self.pub_motor2_2, self.pub_motor2_3, self.pub_motor3_1, self.pub_motor3_2,
                    self.pub_motor3_3, self.pub_motor4_1, self.pub_motor4_2, self.pub_motor4_3,
                    self.pub_motor5_1, self.pub_motor5_2, self.pub_motor5_3, self.pub_motor6_1,
                    self.pub_motor6_2, self.pub_motor6_3]
        #力関係
        self.force_param = [[0 for i in range(3)] for j in range(self._link_num+1)]
        #0:力がかかってない 1:かかっている
        self.force_flag = False
        #0:モーターが動いている 1:動いてない
        self.m_flag = False
        s1 = '/ft_'
        self.ft_1_1 = rospy.Subscriber(s1+'1_1', WrenchStamped, self._ft_1_1_call, queue_size = 1)
        self.ft_1_2 = rospy.Subscriber(s1+'1_2', WrenchStamped, self._ft_1_2_call, queue_size = 1)
        self.ft_1_3 = rospy.Subscriber(s1+'1_3', WrenchStamped, self._ft_1_3_call, queue_size = 1)
        self.ft_2_1 = rospy.Subscriber(s1+'2_1', WrenchStamped, self._ft_2_1_call, queue_size = 1)
        self.ft_2_2 = rospy.Subscriber(s1+'2_2', WrenchStamped, self._ft_2_2_call, queue_size = 1)
        self.ft_2_3 = rospy.Subscriber(s1+'2_3', WrenchStamped, self._ft_2_3_call, queue_size = 1)
        self.ft_3_1 = rospy.Subscriber(s1+'3_1', WrenchStamped, self._ft_3_1_call, queue_size = 1)
        self.ft_3_2 = rospy.Subscriber(s1+'3_2', WrenchStamped, self._ft_3_2_call, queue_size = 1)
        self.ft_3_3 = rospy.Subscriber(s1+'3_3', WrenchStamped, self._ft_3_3_call, queue_size = 1)
        self.ft_4_1 = rospy.Subscriber(s1+'4_1', WrenchStamped, self._ft_4_1_call, queue_size = 1)
        self.ft_4_2 = rospy.Subscriber(s1+'4_2', WrenchStamped, self._ft_4_2_call, queue_size = 1)
        self.ft_4_3 = rospy.Subscriber(s1+'4_3', WrenchStamped, self._ft_4_3_call, queue_size = 1)
        self.ft_5_1 = rospy.Subscriber(s1+'5_1', WrenchStamped, self._ft_5_1_call, queue_size = 1)
        self.ft_5_2 = rospy.Subscriber(s1+'5_2', WrenchStamped, self._ft_5_2_call, queue_size = 1)
        self.ft_5_3 = rospy.Subscriber(s1+'5_3', WrenchStamped, self._ft_5_3_call, queue_size = 1)
        self.ft_6_1 = rospy.Subscriber(s1+'6_1', WrenchStamped, self._ft_6_1_call, queue_size = 1)
        self.ft_6_2 = rospy.Subscriber(s1+'6_2', WrenchStamped, self._ft_6_2_call, queue_size = 1)
        self.ft_6_3 = rospy.Subscriber(s1+'6_3', WrenchStamped, self._ft_6_3_call, queue_size = 1)
        self.ft_base = rospy.Subscriber(s1+'base', WrenchStamped, self._ft_base_call, queue_size = 1)
        self.force = [self.ft_1_1, self.ft_1_2, self.ft_1_3, self.ft_2_1, self.ft_2_2, self.ft_2_3,
                      self.ft_3_1, self.ft_3_2, self.ft_3_3, self.ft_4_1, self.ft_4_2, self.ft_4_3, 
                      self.ft_5_1, self.ft_5_2, self.ft_5_3, self.ft_6_1, self.ft_6_2, self.ft_6_3,
                      self.ft_base]
        #linkの位置を定義
        #引数:[base, l1_1, l1_2, l1_3, l1_4, ..., l6_3][x, y, z, r, p, y]
        self._link_param = [[     0.0,       0.0,      0.0,        0.0,      -0.000002,  0.1],
                           [ 0.050446,  0.058319, -0.000019,  0.000122, -0.000047, -0.700000],
                           [ 0.108430,  0.127154, -0.000020,  0.349349, -0.000089, -0.699979],
                           [ 0.181063,  0.213382,  0.041032, -1.221084, -0.000142, -0.699857],
                           [ 0.219669,  0.259202, -0.123390, -1.221470, -0.000008, -0.700088],
                           [-0.007485,  0.074622,  0.000008,  0.000070,  0.000083,  0.099944],
                           [-0.016466,  0.164173,  0.000012,  0.349182,  0.000068,  0.099932],
                           [-0.027710,  0.276369,  0.041062, -1.221538,  0.000077,  0.099967],
                           [-0.033697,  0.335945, -0.123375, -1.221603,  0.000070,  0.099931],
                           [-0.055070,  0.047759, -0.000096,  0.000919, -0.000042,  0.900046],
                           [-0.125575,  0.103701, -0.000055,  0.350331,  0.000023,  0.900052],
                           [-0.213882,  0.173770,  0.041099, -1.220339,  0.000129,  0.900098],
                           [-0.260928,  0.211088, -0.123280, -1.220541, -0.000081,  0.000191],
                           [ 0.061001, -0.047157,  0.000098, -0.002268, -0.000004,  0.899880],
                           [ 0.131491, -0.103113,  0.000284, -0.351517, -0.000089,  0.899908],
                           [ 0.219742, -0.173151,  0.041584,  1.219117, -0.000083,  0.899855],
                           [ 0.266946, -0.210604, -0.122716,  1.219251, -0.000019,  0.900047],
                           [ 0.007478, -0.074641,  0.000018, -0.001870, -0.000675,  0.099994],
                           [ 0.014673, -0.164192,  0.000119, -0.351291, -0.000521,  0.099958],
                           [ 0.027692, -0.276324,  0.041381,  1.219401, -0.000428,  0.099941],
                           [ 0.033788, -0.336220, -0.122941,  1.219644, -0.000624,  0.099969],
                           [-0.044456, -0.057730, -0.000048,  0.000696,  0.000007, -0.700041],
                           [-0.102441, -0.126572, -0.000155, -0.347830,  0.000006, -0.699992],
                           [-0.175129, -0.212864,  0.040733,  1.223409, -0.000047, -0.699837],
                           [-0.213511, -0.258448, -0.123809,  1.223300,  0.000034, -0.699862]]
        self.joint_name = []
        for i in range(6):
            for j in range(3):
                self.joint_name.append("Joint" + str(i+1) + "_" + str(j+1))
        self.unpause()

    def reset_flag(self):
        self.force_flag = False

    def reset_model(self):
        self.reset_start_motor()
        self.pause()
        for i in range(25):
            self.link.link_name = self.link_name_all[i]
            self.link.pose.position.x = self._link_param[i][0]
            self.link.pose.position.y = self._link_param[i][1]
            self.link.pose.position.z = self._link_param[i][2]
            t = tf.transformations.quaternion_from_euler(self._link_param[i][3], self._link_param[i][4], self._link_param[i][5])
            self.link.pose.orientation.x = t[0]
            self.link.pose.orientation.y = t[1]
            self.link.pose.orientation.z = t[2]
            self.link.pose.orientation.w = t[3]
            self.link.twist.linear.x = 0.0
            self.link.twist.linear.y = 0.0
            self.link.twist.linear.z = 0.0
            self.link.twist.angular.x = 0.0
            self.link.twist.angular.y = 0.0
            self.link.twist.angular.z = 0.0
            self.link.reference_frame = ''
            self.pose_link.publish(self.link)
        self.unpause()
        rospy.sleep(0.05)
        self.reset_start_motor()
        #self.pub_motor(force_stop=False)
        self.reset_flag()

    def reset_start_motor(self):
        #reset_model用のモーターの初期化
        for i in range(18):
            if i % 3 == 0:
                self.power[i] = 0.0
            elif i % 3 == 1:
                self.power[i] = math.radians(20)
            elif i % 3 == 2:
                self.power[i] = math.radians(-90)
            self.pub[i].publish(self.power[i])
        rospy.sleep(0.01)
        reset_req = SetModelConfigurationRequest()
        reset_req.model_name = 'rokusoku1'
        reset_req.urdf_param_name = 'robot_description'
        joint_name = []
        reset_req.joint_names = self.joint_name
        reset_req.joint_positions = self.power
        res = self.reset_joints(reset_req)
        rospy.sleep(0.01)

    def reset_model_old(self):
        #基準位置に持っていく
        #self.reset_world()
        self.set_start_power()
        self.set.model_name = self.model_name
        self.set.pose.position.x = 0.0 + self.offset_x
        self.set.pose.position.y = 0.0 + self.offset_y
        self.set.pose.position.z = 0.1
        self.set.pose.orientation.x = 0.0
        self.set.pose.orientation.y = 0.0
        self.set.pose.orientation.z = 0.0
        self.set.twist.linear.x = 0.0
        self.set.twist.linear.y = 0.0
        self.set.twist.linear.z = 0.0
        self.set.twist.angular.x = 0.0
        self.set.twist.angular.y = 0.0
        self.set.twist.angular.z = 0.0
        self.set.reference_frame = ''
        self.set_pub.publish(self.set)
        self.set_start_power()
        rospy.sleep(0.2)
        #ロボットを初期の状態に持っていく
        self.set_start_power()

    def pub_motor(self, force_stop = True):
        #現在のself.power変数の値をpublish
        for i in range(self._joint_num):
            self.pub[i].publish(self.power[i])
        flag = 0
        t = 0
        loop_start = time.time()
        while(1):
            flag = 0
            self.get_joint_ang()
            for i in range(self._joint_num):
                #角度ごとに、ある一定の角度誤差以下ではなかったら終わってないフラグを立てる 
                if( abs( abs(self.power[i]) - abs(self.now_ang[i]) ) > 0.05):
                    flag = 1
            if flag == 0:
                #ここで終了 ただしモーターが動き終わってからすぐ動くのは不自然なため、delayをつけた
                rospy.sleep(self._MOTER_DELAY)
                return
            loop_time = time.time() - loop_start
            #print(loop_time)
            #なにかにあたっていたら また、力があたった時とめることを許したら
            self.force_detection()
            if self.force_flag == True and force_stop == True:
                return
            if loop_time > 1.0:
                #print("old")
                self.pub_motor_old()
            if loop_time > 2.0:
                #2秒動かなかったら
                #print("moter error!")
                return

    def pub_motor_old(self):
        self.pub_motor1_1.publish(self.power[0])
        self.pub_motor1_2.publish(self.power[1])
        self.pub_motor1_3.publish(self.power[2])
        self.pub_motor2_1.publish(self.power[3])
        self.pub_motor2_2.publish(self.power[4])
        self.pub_motor2_3.publish(self.power[5])
        self.pub_motor3_1.publish(self.power[6])
        self.pub_motor3_2.publish(self.power[7])
        self.pub_motor3_3.publish(self.power[8])
        self.pub_motor4_1.publish(self.power[9])
        self.pub_motor4_2.publish(self.power[10])
        self.pub_motor4_3.publish(self.power[11])
        self.pub_motor5_1.publish(self.power[12])
        self.pub_motor5_2.publish(self.power[13])
        self.pub_motor5_3.publish(self.power[14])
        self.pub_motor6_1.publish(self.power[15])
        self.pub_motor6_2.publish(self.power[16])
        self.pub_motor6_3.publish(self.power[17])
        #print("aaa")
        rospy.sleep(1.0)
 
    def set_start_power(self):
        for i in range(18):
            if i % 3 == 0:
                self.power[i] = 0.0
            elif i % 3 == 1:
                self.power[i] = math.radians(20)
            elif i % 3 == 2:
                self.power[i] = math.radians(-90)
        self.pub_motor(force_stop = False)

    def get_pos(self):
        #各Linkの位置を取得

        reference_frame = ''

        #baseだけ特別処理        
        base_pos_temp = self.posecall(self.link_name[0],reference_frame)
        self.base_x = base_pos_temp.link_state.pose.position.x
        self.base_y = base_pos_temp.link_state.pose.position.y
        self.base_z = base_pos_temp.link_state.pose.position.z
        x_ori  = base_pos_temp.link_state.pose.orientation.x        
        y_ori  = base_pos_temp.link_state.pose.orientation.y        
        z_ori  = base_pos_temp.link_state.pose.orientation.z        
        w_ori  = base_pos_temp.link_state.pose.orientation.w        
        #これでroll, pitch, yawの情報がもらえる
        t = tf.transformations.euler_from_quaternion((x_ori, y_ori, z_ori, w_ori))
        self.roll = t[0]
        self.pitch = t[1]
        self.yaw = t[2]

        link_pos_temp = [0]*18
        link_pos_temp[0] = self.posecall(self.link_name[1],reference_frame)
        link_pos_temp[1] = self.posecall(self.link_name[2],reference_frame)
        link_pos_temp[2] = self.posecall(self.link_name[3],reference_frame)
        link_pos_temp[3] = self.posecall(self.link_name[4],reference_frame)
        link_pos_temp[4] = self.posecall(self.link_name[5],reference_frame)
        link_pos_temp[5] = self.posecall(self.link_name[6],reference_frame)
        link_pos_temp[6] = self.posecall(self.link_name[7],reference_frame)
        link_pos_temp[7] = self.posecall(self.link_name[8],reference_frame)
        link_pos_temp[8] = self.posecall(self.link_name[9],reference_frame)
        link_pos_temp[9] = self.posecall(self.link_name[10],reference_frame)
        link_pos_temp[10] = self.posecall(self.link_name[11],reference_frame)
        link_pos_temp[11] = self.posecall(self.link_name[12],reference_frame)
        link_pos_temp[12] = self.posecall(self.link_name[13],reference_frame)
        link_pos_temp[13] = self.posecall(self.link_name[14],reference_frame)
        link_pos_temp[14] = self.posecall(self.link_name[15],reference_frame)
        link_pos_temp[15] = self.posecall(self.link_name[16],reference_frame)
        link_pos_temp[16] = self.posecall(self.link_name[17],reference_frame)
        link_pos_temp[17] = self.posecall(self.link_name[18],reference_frame)

        for i in range(18):
            for j in range(3):
                if j == 0: self.link_pos[i][j] = link_pos_temp[i].link_state.pose.position.x
                if j == 1: self.link_pos[i][j] = link_pos_temp[i].link_state.pose.position.y
                if j == 2: self.link_pos[i][j] = link_pos_temp[i].link_state.pose.position.z

        #print(self.linkcall1_1.link_state.pose.position.x)

    def get_joint_ang(self):
        #現在のモーター角度を取得
        #rosserviceの/gazebo/get_joint_propertiesを使用
        ang = []
        for i in range(18):
            #Joint2_2など そのまま取得するとtuple型のものが出てくるため、数値だけlistに変換
            a = self.jointcall(self.joint_name[i]).position
            a = list(a)
            ang.append(a[0])
        self.now_ang = ang
  
    def get_center(self):
        #重心を取得
        MB = 0.3
        ML1 = 0.06
        ML2 = 0.08
        ML3 = 0.12
        MS = MB + ML1*6 + ML2*6 + ML3*6
        
        self.get_pos()

        x_pos_sum = 0 + base_x
        for i in range(18):
            x_pos_sum += self.link_pos[i][0]
        y_pos_sum = 0 + base_y
        for i in range(18):
            y_pos_sum += self.link_pos[i][1]
        z_pos_sum = 0 + base_z
        for i in range(18):
            z_pos_sum += self.link_pos[i][2]

        self.center_x = x_pos_sum / MM
        self.center_y = y_pos_sum / MM 
        self.center_z = z_pos_sum / MM 

    #力関係の関数
    def _ft_1_1_call(self, msg):
        self.force_param[0][0] =msg.wrench.force.x
        self.force_param[0][1] =msg.wrench.force.y
        self.force_param[0][2] =msg.wrench.force.z

    def _ft_1_2_call(self, msg):
        self.force_param[1][0] =msg.wrench.force.x
        self.force_param[1][1] =msg.wrench.force.y
        self.force_param[1][2] =msg.wrench.force.z

    def _ft_1_3_call(self, msg):
        self.force_param[2][0] =msg.wrench.force.x
        self.force_param[2][1] =msg.wrench.force.y
        self.force_param[2][2] =msg.wrench.force.z

    def _ft_2_1_call(self, msg):
        self.force_param[3][0] =msg.wrench.force.x
        self.force_param[3][1] =msg.wrench.force.y
        self.force_param[3][2] =msg.wrench.force.z

    def _ft_2_2_call(self, msg):
        self.force_param[4][0] =msg.wrench.force.x
        self.force_param[4][1] =msg.wrench.force.y
        self.force_param[4][2] =msg.wrench.force.z

    def _ft_2_3_call(self, msg):
        self.force_param[5][0] =msg.wrench.force.x
        self.force_param[5][1] =msg.wrench.force.y
        self.force_param[5][2] =msg.wrench.force.z

    def _ft_3_1_call(self, msg):
        self.force_param[6][0] =msg.wrench.force.x
        self.force_param[6][1] =msg.wrench.force.y
        self.force_param[6][2] =msg.wrench.force.z

    def _ft_3_2_call(self, msg):
        self.force_param[7][0] =msg.wrench.force.x
        self.force_param[7][1] =msg.wrench.force.y
        self.force_param[7][2] =msg.wrench.force.z

    def _ft_3_3_call(self, msg):
        self.force_param[8][0] =msg.wrench.force.x
        self.force_param[8][1] =msg.wrench.force.y
        self.force_param[8][2] =msg.wrench.force.z

    def _ft_4_1_call(self, msg):
        self.force_param[9][0] =msg.wrench.force.x
        self.force_param[9][1] =msg.wrench.force.y
        self.force_param[9][2] =msg.wrench.force.z

    def _ft_4_2_call(self, msg):
        self.force_param[10][0] =msg.wrench.force.x
        self.force_param[10][1] =msg.wrench.force.y
        self.force_param[10][2] =msg.wrench.force.z

    def _ft_4_3_call(self, msg):
        self.force_param[11][0] =msg.wrench.force.x
        self.force_param[11][1] =msg.wrench.force.y
        self.force_param[11][2] =msg.wrench.force.z

    def _ft_5_1_call(self, msg):
        self.force_param[12][0] =msg.wrench.force.x
        self.force_param[12][1] =msg.wrench.force.y
        self.force_param[12][2] =msg.wrench.force.z

    def _ft_5_2_call(self, msg):
        self.force_param[13][0] =msg.wrench.force.x
        self.force_param[13][1] =msg.wrench.force.y
        self.force_param[13][2] =msg.wrench.force.z

    def _ft_5_3_call(self, msg):
        self.force_param[14][0] =msg.wrench.force.x
        self.force_param[14][1] =msg.wrench.force.y
        self.force_param[14][2] =msg.wrench.force.z

    def _ft_6_1_call(self, msg):
        self.force_param[15][0] =msg.wrench.force.x
        self.force_param[15][1] =msg.wrench.force.y
        self.force_param[15][2] =msg.wrench.force.z

    def _ft_6_2_call(self, msg):
        self.force_param[16][0] =msg.wrench.force.x
        self.force_param[16][1] =msg.wrench.force.y
        self.force_param[16][2] =msg.wrench.force.z

    def _ft_6_3_call(self, msg):
        self.force_param[17][0] =msg.wrench.force.x
        self.force_param[17][1] =msg.wrench.force.y
        self.force_param[17][2] =msg.wrench.force.z

    def _ft_base_call(self, msg):
        self.force_param[18][0] =msg.wrench.force.x
        self.force_param[18][1] =msg.wrench.force.y
        self.force_param[18][2] =msg.wrench.force.z

    #力を検出
    def force_detection(self):
        force_limit = 50
        f = []
        force_max = np.max(np.abs(self.force_param))
        #print(force_max)
        if self.m_flag == True and self.force_flag == False:
            if force_max > force_limit:
                #print(force_max)
                self.force_flag = True
            else:
                self.force_flag = False
