#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2,atan
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus,ObjectStatusList
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from std_msgs.msg import String


class recong_pub():
    def __init__(self):
        rospy.init_node('recong_pub', anonymous=True)

        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.recong_pub = rospy.Publisher('recong', String, queue_size=1)

        self.recong_msg = 'lattice'
        self.current_postion = Point()

        # # 어린이 보호구역
        # child_protect_s = (92, 1141)
        # child_protect_e = (127, 1203)

        # # 장애물 구간
        # obs_s = (99, 1253)
        # obs_e = (137, 1353)

        # # 갓길주차 구간
        # park_s = (74, 1547)
        # park_e = (80, 1605)
        
        # 차선변경1
        lane_change_1 = (197, 1772)
        # 차선변경2
        lane_change_2 = (202, 1606)
        
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():

            # # 어린이 보호구역
            # dis = sqrt(pow(self.current_postion.x - child_protect_s[0], 2) + pow(self.current_postion.y - child_protect_s[1], 2))
            # if dis < 2:
            #     self.recong_msg = 'child'
            # dis = sqrt(pow(self.current_postion.x - child_protect_e[0], 2) + pow(self.current_postion.y - child_protect_e[1], 2))
            # if dis < 2:
            #     self.recong_msg = 'basic'
            
            # # 장애물 구간
            # dis = sqrt(pow(self.current_postion.x - obs_s[0], 2) + pow(self.current_postion.y - obs_s[1], 2))
            # if dis < 2:
            #     self.recong_msg = 'lattice'
            # dis = sqrt(pow(self.current_postion.x - obs_e[0], 2) + pow(self.current_postion.y - obs_e[1], 2))
            # if dis < 2:
            #     self.recong_msg = 'basic'
            
            # # 갓길주차 구간
            # dis = sqrt(pow(self.current_postion.x - park_s[0], 2) + pow(self.current_postion.y - park_s[1], 2))
            # if dis < 2:
            #     self.recong_msg = 'lattice'
            # dis = sqrt(pow(self.current_postion.x - park_e[0], 2) + pow(self.current_postion.y - park_e[1], 2))
            # if dis < 2:
            #     self.recong_msg = 'basic'

            # 차선 변경
            dis1 = sqrt(pow(self.current_postion.x - lane_change_1[0], 2) + pow(self.current_postion.y - lane_change_1[1], 2))
            dis2 = sqrt(pow(self.current_postion.x - lane_change_2[0], 2) + pow(self.current_postion.y - lane_change_2[1], 2))
            if dis1 < 5 or dis2 < 5:
                self.recong_msg = 'lane_change'
            if dis1 > 5 and dis2 > 5 and self.recong_msg == 'lane_change':
                self.recong_msg = 'basic'

            rospy.loginfo(self.recong_msg)
            self.recong_pub.publish(self.recong_msg)

            rate.sleep()

    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

if __name__ == '__main__':
    try:
        recong = recong_pub()
    except rospy.ROSInterruptException:
        pass
