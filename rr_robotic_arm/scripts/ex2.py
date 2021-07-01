#!/usr/bin/env python
"""
Script for Exercise 2
"""
# rospy - ROS Python API
import rospy
import math
import time
import tf
from time import time
import numpy as np
from time import sleep

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from math import sin,cos,atan2,sqrt,fabs,pi

length_rigid_seg1 = 1  # in m
length_rigid_seg2 = 1  # in m


def rrbot_joint_torques_publisher():
	global pub1, pub2


	# initialize our ROS node, registering it with the Master and for controlling joint1 and joint2 positions.
	rospy.init_node('rrbot_joint_torque_node', anonymous=True)

	#Define publishers for each joint position controller commands.
	pub1 = rospy.Publisher('/rrbot/joint_base_mid_torque_controller/command', Float64, queue_size=100)
	pub2 = rospy.Publisher('/rrbot/joint_mid_top_torque_controller/command', Float64, queue_size=100)

	#rate = rospy.Rate(100) #100 Hz
	rospy.Subscriber('/rrbot/joint_states', JointState, callback)


def callback(message):
	global torque1, torque2, pub1, pub2
	joint_base_mid_angle = message.position[0]
	joint_mid_top_angle = message.position[1]

	joint_base_mid_vel = message.velocity[0]
	joint_mid_top_vel = message.velocity[1]
	sleep(0.1)


	print "The joint angles for the base_mid and mid_top joints are:", joint_base_mid_angle, joint_mid_top_angle, "respectively" 
	sleep(0.1)
	print "The joint velocities for the base_mid and mid_top joints are:", joint_base_mid_vel, joint_mid_top_vel, "respectively"

	q_y = (length_rigid_seg1*cos(joint_base_mid_angle)) + (length_rigid_seg2*cos((joint_base_mid_angle+joint_mid_top_angle)))
	q_x = (length_rigid_seg1*sin(joint_base_mid_angle)) + (length_rigid_seg2*sin((joint_base_mid_angle+joint_mid_top_angle)))

	print ("The cartesian coordinates of the end effector are:", q_x, q_y, "respectively" )
	sleep(0.1)

	if abs(joint_base_mid_angle)< 3.0 and abs(joint_mid_top_angle)< 3.0  :
		pub1.publish(2.5*torque1)
		pub2.publish(1.2*torque2)
	else:
		pub1.publish(0.0)
		pub2.publish(0.0)
			

	




#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
	torque1 = (input("Enter the desired torque for base_mid (red_green) joint in Nm : "))
	torque2 = (input("Enter the desired torque for mid_top (green_blue) joint in Nm : "))

	try: 
		rrbot_joint_torques_publisher()
	except rospy.ROSInterruptException: 
		pass
	rospy.spin()