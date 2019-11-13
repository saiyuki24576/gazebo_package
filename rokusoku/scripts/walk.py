#!/usr/bin/env python
# coding: utf-8

from __future__ import print_function

import Robot
import numpy as np
import rospy
import roslib
import time
import math
import random
import sys
import os
import datetime
import pickle
import argparse
from statistics import mean


from std_msgs.msg import Int64
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
import tf

JOINT_NUM = 18
USE_JOINT = 12

#クラスを継承している
class SixlegClass(Robot.SixlegController, object):
    def __init__(self, robotID, posofsetx, posofsety):
        super(SixlegClass, self).__init__(robotID, posofsetx, posofsety)
        #モーターのパワーを出力するもの　1-1:0 1-2:1 1-3:2 2-1:3・・・6-3:17となっている
        #self.power = [0]*JOINT_NUM

def timer_callback(event):
    try:
        c.get_pos()
        c.force_detection()
    except:
        print('service error')


if __name__ == '__main__':
    #ノードの初期化と宣言
    rospy.init_node("robot1", anonymous=False)
    while not rospy.is_shutdown():
        #クラスを宣言
        c = SixlegClass(1, 0, 0)
        time.sleep(2.0)
        #一定時間ごとに実行される関数を生成　基本的にはセンサーの情報を取得のために使う
        rospy.Timer(rospy.Duration(0.02), timer_callback)
        c.reset_model()

        for i in range(3):
                c.power[i*6] = math.radians(-20)
                c.power[i*6+1] = math.radians(30)
                c.power[i*6+2] = math.radians(-90)
        while(1):
            for i in range(3):
                c.power[i*6] = math.radians(0)
                c.power[i*6+1] = math.radians(-20)
                c.power[i*6+2] = math.radians(-90)

            for i in range(3):
                c.power[i*6+3] = math.radians(-20)
                c.power[i*6+1+3] = math.radians(30)
                c.power[i*6+2+3] = math.radians(-90)
            c.pub_motor()


            for i in range(3):
                c.power[i*6] = math.radians(-20)
                c.power[i*6+1] = math.radians(30)
                c.power[i*6+2] = math.radians(-90)

            for i in range(3):
                c.power[i*6+3] = math.radians(0)
                c.power[i*6+1+3] = math.radians(-20)
                c.power[i*6+2+3] = math.radians(-90)
            c.pub_motor()
