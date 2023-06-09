#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
import rospy
import rospkg
import tf
from math import cos,sin,pi,sqrt,pow,atan2
import numpy as np
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import ObjectStatus, ObjectStatusList, EgoVehicleStatus
from std_msgs.msg import String

class lc_path_pub :
    def __init__(self):
        rospy.init_node('lc_path_pub', anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        object_topic_name = arg[1]

        rospy.Subscriber(object_topic_name, ObjectStatusList, self.object_info_callback)

        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber('recong', String, self.recong_callback)

        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.local_path_pub = rospy.Publisher('/local_path',Path, queue_size=1)

        self.lc_1=Path()
        self.lc_1.header.frame_id='/map'
        self.lc_2=Path()
        self.lc_2.header.frame_id='/map'

        rospack=rospkg.RosPack()
        pkg_path=rospack.get_path('scenario_1')

        lc_1 = pkg_path + '/path' + '/scenario_global_path.txt'
        self.f=open(lc_1,'r')
        lines = self.f.readlines()
        for line in lines :
            tmp = line.split()
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(tmp[0])
            read_pose.pose.position.y = float(tmp[1])
            read_pose.pose.orientation.w = 1
            self.lc_1.poses.append(read_pose)  
        self.f.close()

        lc_2 = pkg_path + '/path' + '/scenario_global_path.txt'
        self.f=open(lc_2,'r')
        lines = self.f.readlines()
        for line in lines :
            tmp = line.split()
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(tmp[0])
            read_pose.pose.position.y = float(tmp[1])
            read_pose.pose.orientation.w = 1
            self.lc_2.poses.append(read_pose) 
        self.f.close()

        self.is_object_info = False
        self.is_odom = False

        self.local_path_size = 60
        
        self.lane_change = False        
        self.current_lane = 1

        self.recong = String

        global_path = self.lc_1

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            print(self.recong.data)
            if self.is_object_info == True and self.is_odom == True:

                self.local_path_make(global_path)

                currnet_waypoint = self.get_current_waypoint(self.x,self.y,global_path)

                # global_path = self.lc_planning(global_npc_info,local_npc_info,currnet_waypoint,global_path)
                global_path = self.lc_planning(currnet_waypoint,global_path)

                self.local_path_pub.publish(self.local_path_msg)
                self.global_path_pub.publish(global_path)
                
            rate.sleep()

    def recong_callback(self, msg):
        self.recong = msg

    def odom_callback(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)

        _,_,self.vehicle_yaw=tf.transformations.euler_from_quaternion(odom_quaternion)

        self.is_odom = True

    def object_info_callback(self,data): ## Object information Subscriber
        self.is_object_info = True
        self.object_data = data 
    
    def get_current_waypoint(self,x,y,global_path):
        min_dist = float('inf')        
        currnet_waypoint = -1
        for i,pose in enumerate(global_path.poses):
            dx = x - pose.pose.position.x
            dy = y - pose.pose.position.y

            dist = sqrt(pow(dx,2)+pow(dy,2))
            if min_dist > dist :
                min_dist = dist
                currnet_waypoint = i
        return currnet_waypoint
    
    def local_path_make(self,global_path):                
        self.local_path_msg=Path()
        self.local_path_msg.header.frame_id='/map'
        
        x=self.x
        y=self.y
        min_dis=float('inf')
        current_waypoint=-1
        for i,waypoint in enumerate(global_path.poses) :
            distance=sqrt(pow(x-waypoint.pose.position.x,2)+pow(y-waypoint.pose.position.y,2))
            if distance < min_dis :
                min_dis=distance
                current_waypoint=i
        
        if current_waypoint != -1 :
            if current_waypoint + self.local_path_size < len(global_path.poses):
                for num in range(current_waypoint,current_waypoint + self.local_path_size ) :
                    tmp_pose=PoseStamped()
                    tmp_pose.pose.position.x=global_path.poses[num].pose.position.x
                    tmp_pose.pose.position.y=global_path.poses[num].pose.position.y
                    tmp_pose.pose.orientation.w=1
                    self.local_path_msg.poses.append(tmp_pose)                    
            else :
                for num in range(current_waypoint,len(global_path.poses) ) :
                    tmp_pose=PoseStamped()
                    tmp_pose.pose.position.x=global_path.poses[num].pose.position.x
                    tmp_pose.pose.position.y=global_path.poses[num].pose.position.y
                    tmp_pose.pose.orientation.w=1
                    self.local_path_msg.poses.append(tmp_pose)


    def calc_vaild_obj(self,status_msg,object_data):
        
        self.all_object = object_data        
        ego_pose_x = status_msg[0]
        ego_pose_y = status_msg[1]
        ego_heading = status_msg[2]
        
        global_npc_info = []
        local_npc_info  = []

        num_of_object = self.all_object.num_of_npcs        
        if num_of_object > 0:

            #translation
            tmp_theta=ego_heading
            tmp_translation=[ego_pose_x, ego_pose_y]
            tmp_t=np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]],
                            [sin(tmp_theta),  cos(tmp_theta), tmp_translation[1]],
                            [0             ,               0,                  1]])
            tmp_det_t=np.array([[tmp_t[0][0], tmp_t[1][0], -(tmp_t[0][0] * tmp_translation[0] + tmp_t[1][0]*tmp_translation[1])],
                                [tmp_t[0][1], tmp_t[1][1], -(tmp_t[0][1] * tmp_translation[0] + tmp_t[1][1]*tmp_translation[1])],
                                [0,0,1]])

            #npc vehicle ranslation        
            for npc_list in self.all_object.npc_list:
                global_result=np.array([[npc_list.position.x],[npc_list.position.y],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]> 0 :        
                    global_npc_info.append([npc_list.type,npc_list.position.x,npc_list.position.y,npc_list.velocity.x])
                    local_npc_info.append([npc_list.type,local_result[0][0],local_result[1][0],npc_list.velocity.x])    

        return global_npc_info, local_npc_info

    # def lc_planning(self,global_obj,local_obj,currnet_waypoint,global_path):
    def lc_planning(self,currnet_waypoint,global_path):

        lane_change_distance = 30 * 2 # (point-to-point distance 0.5m)

        print(self.recong.data)
        if self.lane_change == True:
            if currnet_waypoint + self.local_path_size > len(global_path.poses):
                self.lane_change = False
                if self.current_lane == 1:
                    global_path = self.lc_1
                elif self.current_lane == 2:
                    global_path = self.lc_2
        # else:
        #     self.check_object(self.local_path_msg,global_obj,local_obj)

        elif self.recong.data == 'lane_change': # self.object[0] == True:
            if self.current_lane != 1:
                self.current_lane = 1
                start_point         = self.lc_2.poses[currnet_waypoint]  
                end_waypoint_idx = self.get_current_waypoint(self.lc_1.poses[currnet_waypoint+lane_change_distance].pose.position.x,self.lc_1.poses[currnet_waypoint+lane_change_distance].pose.position.y,self.lc_1)
                end_point           = self.lc_1.poses[end_waypoint_idx]
                start_next_point    = self.lc_2.poses[currnet_waypoint+5]  
                global_path = self.getLaneChangePath(self.lc_2,self.lc_1,start_point,end_point,start_next_point, end_waypoint_idx)
                self.lane_change = True
                self.object=[False,0]
            else:
                self.current_lane = 2
                start_point         = self.lc_1.poses[currnet_waypoint]  
                end_waypoint_idx = self.get_current_waypoint(self.lc_2.poses[currnet_waypoint+lane_change_distance].pose.position.x,self.lc_2.poses[currnet_waypoint+lane_change_distance].pose.position.y,self.lc_2)
                end_point           = self.lc_2.poses[end_waypoint_idx]
                start_next_point    = self.lc_1.poses[currnet_waypoint+5]  
                global_path = self.getLaneChangePath(self.lc_1,self.lc_2,start_point,end_point,start_next_point, end_waypoint_idx)
                self.lane_change = True
                self.object=[False,0]

        return global_path

    # def check_object(self,ref_path,global_vaild_object,local_vaild_object):
    #     self.object=[False,0]
    #     if len(global_vaild_object) >0  :
    #         min_rel_distance = float('inf')
    #         for i in range(len(global_vaild_object)):
    #             for path in ref_path.poses :   
    #                 if global_vaild_object[i][0]==1 or global_vaild_object[i][0]==2 :  
    #                     dis = sqrt(pow(global_vaild_object[i][1] - path.pose.position.x, 2) + pow(global_vaild_object[i][2] - path.pose.position.y, 2))
    #                     if dis<2.5:
    #                         rel_distance = sqrt(pow(local_vaild_object[i][1], 2) + pow(local_vaild_object[i][2], 2))
    #                         if rel_distance < min_rel_distance:
    #                             min_rel_distance = rel_distance
    #                             self.object=[True,i]

    def getLaneChangePath(self,ego_path,lc_path,start_point,end_point,start_next_point, end_waypoint_idx): ## 
        out_path=Path()  
        out_path.header.frame_id='/map'

        translation = [start_point.pose.position.x, start_point.pose.position.y]
        theta       = atan2(start_next_point.pose.position.y-start_point.pose.position.y,start_next_point.pose.position.x-start_point.pose.position.x)

        trans_matrix = np.array([   [cos(theta), -sin(theta), translation[0]],
                                    [sin(theta), cos(theta), translation[1]],
                                    [          0,             0,              1] ])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        # '''

        world_end_point=np.array([[end_point.pose.position.x],[end_point.pose.position.y],[1]])
        local_end_point=det_trans_matrix.dot(world_end_point)

        waypoints_x=[]
        waypoints_y=[]
        x_interval = 0.5 # 생성할 Path 의 Point 간격을 0.5 로 한다.
        x_start=0
        x_end=local_end_point[0][0]

        y_start = 0.0
        y_end = local_end_point[1][0]

        x_num = x_end/x_interval
        for i in range(x_start,int(x_num)) : 
            waypoints_x.append(i*x_interval)

        d = 0
        c = 0
        b = 3 * y_end / pow(x_end, 2)
        a = -2 * y_end / pow(x_end, 3)

        for i in waypoints_x:
            # 3 차 방정식 수식을 작성한다. (f(x) = a*x^3 + b*x^2 + c*x + d)
            result = a * pow(i, 3) + b * pow(i, 2) + c * i + d 
            waypoints_y.append(result)

        for i in range(0,len(waypoints_y)) :
            local_result = np.array([[waypoints_x[i]],[waypoints_y[i]],[1]])
            global_result = trans_matrix.dot(local_result)

            read_pose=PoseStamped()
            read_pose.pose.position.x = global_result[0][0]
            read_pose.pose.position.y = global_result[1][0]
            read_pose.pose.position.z = 0.
            read_pose.pose.orientation.x = 0
            read_pose.pose.orientation.y = 0
            read_pose.pose.orientation.z = 0
            read_pose.pose.orientation.w = 1
            out_path.poses.append(read_pose)

        for k in range(end_waypoint_idx,end_waypoint_idx+40):
            read_pose=PoseStamped()
            read_pose.pose.position.x = lc_path.poses[k].pose.position.x
            read_pose.pose.position.y = lc_path.poses[k].pose.position.y
            read_pose.pose.position.z = 0
            read_pose.pose.orientation.x = 0
            read_pose.pose.orientation.y = 0
            read_pose.pose.orientation.z = 0
            read_pose.pose.orientation.w = 1
            out_path.poses.append(read_pose)

        return out_path

if __name__ == '__main__':
    try:
        test_track=lc_path_pub()
    except rospy.ROSInterruptException:
        pass
