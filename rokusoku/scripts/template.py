#!/usr/bin/env python
# coding: utf-8

#作成したmodule いろいろと入っている。
import Robot

import numpy as np
import rospy
import roslib
import time
import math
import random

def timer_callback(event):
    #各Linkの位置と、ベースのroll,pitch,yowを取得
    rob.get_pos()
    
if __name__ == '__main__':
    #ノードの初期化と宣言
    rospy.init_node("rokusoku_py")
    #クラスを宣言
    #クラスの生成 Robot.pyのRobotクラスを呼び出している。 引数はrobotの番号、xの座標、yの座標
    rob = Robot.SixlegController(1, 0, 0)
    #一定時間ごとに実行される関数を生成　基本的にはセンサーの情報を取得のために使う
    rospy.Timer(rospy.Duration(0.05), timer_callback)
    #最初だけsleepをかけないとなぜか一部動かないものがある
    time.sleep(2.0)
    #main部分
    while not rospy.is_shutdown():
        #モーターの位置をpublish
        rob.power[0] = math.radians(20)
        rob.power[1] = math.radians(50)
        rob.pub_motor()
        #ベース座標はよくつかうので特別な変数で定義してある
        X = rob.base_x
        print X
        #link2-1のx,y,z座標を表示する
        print rob.link_pos[3][0],rob.link_pos[3][1], rob.link_pos[3][2]
        #ロボットの初期状態のモーター角度に戻す
        rob.set_start_power()
        if random.randint(0, 5) % 5 == 0:
            #ロボットを初期状態へ戻す
            rob.reset_model()
        else:
            time.sleep(0.5)

    rospy.sleep()
