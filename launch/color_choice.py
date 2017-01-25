#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
import rospy
from vrep_common.srv import simRosStartSimulation, simRosGetObjectHandle
from std_msgs.msg import String
from pylab import tan, pi, randint

# main
rospy.init_node('bridge')

# wait for simulation to be ready
start = rospy.ServiceProxy('/vrep/simRosStartSimulation', simRosStartSimulation)
start.wait_for_service()
start.call()

# color to publish
color = String() 
col_pub = rospy.Publisher('/color_to_detect', String, queue_size=10)

while not rospy.has_param('/colors'):
    rospy.sleep(1)
colors = rospy.get_param('/colors')
colors = colors.keys()

count = 0
color_idx = 0
while not rospy.is_shutdown():

    # colors
    if count == 0:
        # always change color
	color_idx = (color_idx + randint(1,len(colors))) % len(colors)
        color.data = colors[color_idx]
    count = (count+1)%5
    
    col_pub.publish(color)

    rospy.sleep(1)

