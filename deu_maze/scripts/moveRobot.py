#!/usr/bin/env python
# -*-coding: utf-8 -*-
# BEGIN ALL
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

"""
터틀봇의 Odom 데이터를 이용하여 

터틀봇의 방향을 위, 아래, 왼쪽, 오른쪽으로 설정한다.
"""

class MoveRobot:
    def __init__(self):
        self.robot_state_drive = False
        self.robot_state_rotate = False
        self.finish_action = True
        #rospy.init_node('move_robot')
        sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
        self.pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.r = rospy.Rate(10)

        #회전을 위한 선언 및 초기화
        self.roll = self.pitch = self.yaw = 0.0
        self.target = 0
        self.command =Twist()

        self.driving_forward = True 
        self.light_change_time = rospy.Time.now()

    def get_rotation(self, msg):
        self.roll, self.pitch, self.yaw
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)

    def setRight(self):
        self.target = 0 # right, self.yaw=0.0
        self.robot_state_rotate = True 
        self.finish_action = True 

    def setLeft(self):
        self.target = 180 # left, self.yaw=3.14
        self.robot_state_rotate = True 
        self.finish_action = True 

    def setUpward(self):
        self.target = 90 # up, yqw=1.57
        self.robot_state_rotate = True 
        self.finish_action = True 

    def setDownward(self):
        self.target = -90 # bottom, self.yaw=-1.57
        self.robot_state_rotate = True 
        self.finish_action = True 

    def setGo(self):
        self.light_change_time = rospy.Time.now() + rospy.Duration(0.05)
        #self.light_change_time = rospy.Time.now() + rospy.Duration(0.34)
        self.robot_state_drive = True 
        self.finish_action = True 

    def rotate(self):
        if self.target == 180 and self.yaw <= 3.143 and self.yaw >=3.14: # left
            self.command.angular.z = 0
            self.robot_state_rotate = False
            self.finish_action = False
        if self.target == 180 and self.yaw >= 3.143: # left
            self.command.angular.z = -7.35 
        if self.target == 180 and self.yaw <=3.14: # left
            self.command.angular.z = 7.35 
        if self.target == 180 and self.yaw < -1: # left
            self.command.angular.z = -7.35 

        if self.target == 90 and self.yaw <= 1.572 and self.yaw >=1.569: # up
            self.command.angular.z = 0
            self.robot_state_rotate = False
            self.finish_action = False
        elif self.target == 90 and self.yaw >= 1.572: # up
            self.command.angular.z = -7.35 
        elif self.target == 90 and self.yaw <=1.569: # up
            self.command.angular.z = 7.35 

        if self.target == -90 and self.yaw >=-1.572 and self.yaw <= -1.569: # bottom
            self.command.angular.z = 0
            self.robot_state_rotate = False
            self.finish_action = False
        elif self.target == -90 and self.yaw <=-1.572: # bottom
            self.command.angular.z = 7.35 
        elif self.target == -90 and self.yaw > 1.579: # bottom
            self.command.angular.z = 7.35 
        elif self.target == -90 and self.yaw >= -1.569: # bottom
            self.command.angular.z = -7.35 

        if self.target == 0 and self.yaw <= 0.001 and self.yaw >= -0.001: # right
            self.command.angular.z = 0
            self.robot_state_rotate = False
            self.finish_action = False
        elif self.target == 0 and self.yaw >= 0.001:# right
            self.command.angular.z = -7.35 
        elif self.target == 0 and self.yaw <= -0.001: # right
            self.command.angular.z = 7.35 

    def go_ahead(self):
        if self.driving_forward:
            self.command.linear.x = 1.0
        else:
            self.command.linear.x = 0
            self.driving_forward = not self.driving_forward
            self.robot_state_drive = False
            self.finish_action = False
        if rospy.Time.now() > self.light_change_time: 
            self.driving_forward = not self.driving_forward

    def execute(self):
       # while not rospy.is_shutdown():
        while(self.finish_action == True):
            if(self.robot_state_drive):
                self.go_ahead()
            elif(self.robot_state_rotate):    
                self.rotate()
            self.pub.publish(self.command)
            self.r.sleep()
