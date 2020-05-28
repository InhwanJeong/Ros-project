#!usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('open_blocking_bar')

open_bar = Twist()
open_bar.angular.y = 0.5
