#!/usr/bin/env python
# -*-coding: utf-8 -*-

import rospy
from deu_maze.msg import LidarMeasure
from geometry_msgs.msg import Twist
from moveRobot import MoveRobot
from nav_msgs.msg import Odometry
from rotateRobot import RotateRobot


"""
터틀봇이 구독받은 scan값에 따라 "전방, 좌, 우, 후방"을 우선순위로 이동하게됩니다.
터틀봇은 이동할 때마다 방향을 stack에 기록합니다.
미로 탈출 후 기록한 방향을 pop 하며 출발점으로 되돌아갑니다.

"""

def get_odom(msg): # 터틀봇이 출구로 빠져나간것을 지속적으로 확인
    global end_point, turtlebot_pose_y, check
    turtlebot_pose_y = msg.pose.pose.position.y
    if(turtlebot_pose_y < -10):
        check = 1
        end_point = True
    

def lidar_data_callback(floatmsg):
    global init_state            # 터틀봇을 초기 입구로 보내는 변수
    global end_point             # 출구에 도착했는지 확인하는 변수
    global robot_state_direction # 로봇의 현재 방향을 저장하는 변수
    global save_direction        # robot_state_direction을 저장하는 스택
    global turtlebot_pose_y      # 터틀봇의 pose.y의 위치를 확인
    global check                 # 터틀봇 pose.y에 따른 이벤트 발생 변수
    """
    데이터 확인이 필요하다면 주석을 풀어주세요.
    print("ahead: %s" % floatmsg.range_ahead)
    print("left: %s" % floatmsg.range_left)
    print("rear: %s" % floatmsg.range_rear)
    print("right: %s" % floatmsg.range_right)
    print("") 
    print save_direction 
    print("") 
    """
    
    if(check == 1 and turtlebot_pose_y < -10): # 미로 탈출시 다시 미로로 진입
        mr.setUpward()
        mr.execute()
        mr.setGo()
        mr.execute()

    elif(end_point and turtlebot_pose_y > -9.5 ): # 쌓아놓은 스택을 pop 하며 입구로 이동
        if not save_direction:
            msg = rospy.wait_for_message("lidar_measure", LidarMeasure )
        if(check == 1):
            for i in range(11):
                save_direction.pop()
            check = 0
        if(len(save_direction) == 150):
            save_direction.pop()
            save_direction.pop()

        robot_state_direction = (save_direction.pop() + 2) %4
        if(robot_state_direction == 0):
            mr.setDownward()
        elif(robot_state_direction == 1):
            mr.setRight()
        elif(robot_state_direction == 2):
            mr.setUpward()
        elif(robot_state_direction == 3):
            mr.setLeft()
        mr.execute()
        mr.setGo()
        mr.execute()

    elif(init_state): # 초기 시작 미로 안으로 진입
        mr.setDownward()
        mr.execute()
        robot_state_direction = 0
        mr.setGo()
        mr.execute()
        init_state = False
    else:            # 초기 상태 후 미로를 탈출하는 알고리즘
        if(robot_state_direction == 0):    # 아래
            if(floatmsg.range_ahead < 0.75):
                if(floatmsg.range_right > 1.2):
                    mr.setLeft()
                    robot_state_direction = 3
                elif(floatmsg.range_left > 1.2):
                    mr.setRight()
                    robot_state_direction = 1
                else:
                    mr.setUpward()
                    robot_state_direction = 2
            else:    
                mr.setGo()
            mr.execute()

        elif(robot_state_direction == 1):   # 오른쪽
            if(floatmsg.range_ahead < 0.75):
                if(floatmsg.range_left > 1.2):
                    mr.setUpward()
                    robot_state_direction = 2
                elif(floatmsg.range_right > 1.2):
                    mr.setDownward()
                    robot_state_direction = 0
                else:
                    mr.setLeft()
                    robot_state_direction = 3
            else:    
                mr.setGo()
            mr.execute()
        elif(robot_state_direction == 2):   # 위     
            if(floatmsg.range_ahead < 0.75):
                if(floatmsg.range_right > 1.2):
                    mr.setRight()
                    robot_state_direction = 1
                elif(floatmsg.range_left > 1.2):
                    mr.setLeft()
                    robot_state_direction = 3
                else:
                    mr.setDownward()
                    robot_state_direction = 0
            else:    
                mr.setGo()
            mr.execute()
        elif(robot_state_direction == 3):    # 왼쪽  
            if(floatmsg.range_ahead < 0.75):
                if(floatmsg.range_right > 1.2):
                    mr.setUpward()
                    robot_state_direction = 2
                elif(floatmsg.range_left > 1.2):
                    mr.setDownward()
                    robot_state_direction = 0
                else:
                    mr.setRight()
                    robot_state_direction = 1
            else:    
                mr.setGo()
            mr.execute()
        save_direction.append(robot_state_direction)

rospy.init_node('escape_algorithm')

mr = MoveRobot()

robot_state_direction = 1 # 0 아래, 1 왼쪽, 2 위, 3 오른쪽  -> 터틀봇 기준
save_direction = []
init_state = True
end_point = False
check = 0
turtlebot_pose_y = 0

scan_sub = rospy.Subscriber('lidar_measure', LidarMeasure , lidar_data_callback, queue_size =1)
odom_sub = rospy.Subscriber ('/odom', Odometry, get_odom, queue_size = 1)


rospy.spin()

