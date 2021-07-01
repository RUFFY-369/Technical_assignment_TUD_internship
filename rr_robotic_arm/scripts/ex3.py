#!/usr/bin/env python

"""
Script for Exercise 3
"""
# rospy - ROS Python API
import rospy
import math
import time
import tf
from time import time
import numpy as np
from time import sleep
import numpy as np

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from math import sin,cos,atan2,sqrt,fabs,pi,acos,asin, pow

length_rigid_seg1 = 1
length_rigid_seg2 = 1


def inverse_kinematics_and_joint_publisher():

	#Initiate node for controlling joint1 and joint2 positions.
	rospy.init_node('rrbot_joint_positions_node', anonymous=True)

	#Define publishers for each joint position controller commands.
	pub1 = rospy.Publisher('/rrbot/joint_base_mid_position_controller/command', Float64, queue_size=10)
	pub2 = rospy.Publisher('/rrbot/joint_mid_top_position_controller/command', Float64, queue_size=10)

	rate = rospy.Rate(100) #100 Hz

    


	#While loop to have joints follow a certain position, while rospy is not shutdown.
	i = 0
	while not rospy.is_shutdown():

		#Analytical approach for Inverse Kinematics of the Robotic Arm

		desired_cart_x = input("Enter your desired y cartesian coordinate (Green Line as seen) value w.r.t base (red link) with gazebo world frame (Right Hand Coordinate system) as reference: ")
		desired_cart_y = input("Enter your  desired x cartesian coordinate value (Red Line as seen) w.r.t base (red link) with gazebo world frame (Right Hand Coordinate system) as reference: ")

		joint_mid_top_angle = acos((pow(desired_cart_x,2)+ pow(desired_cart_y,2) - pow(length_rigid_seg1,2) - pow(length_rigid_seg2,2))/(2*length_rigid_seg1*length_rigid_seg2))
        
		psi = atan2((length_rigid_seg1 + (length_rigid_seg2*cos(joint_mid_top_angle))),(length_rigid_seg2*sin(joint_mid_top_angle)))

		joint_base_mid_angle = psi - atan2(desired_cart_x,desired_cart_y)

		print "The joint angles for the base_mid and mid_top joints are:", joint_base_mid_angle, joint_mid_top_angle, "respectively"


		#Publish the required configuration to each joint.
		pub1.publish(joint_base_mid_angle)
		pub2.publish(joint_mid_top_angle)

		# TWO POSSIBLE SOLUTIONS
		# pub1.publish(-joint_base_mid_angle)
		# pub2.publish(-joint_mid_top_angle)

		i = i+1 #increment i

		rate.sleep() #sleep for rest of rospy.Rate(100)


#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
	try: 
		inverse_kinematics_and_joint_publisher()
	except rospy.ROSInterruptException: 
		pass
	rospy.spin()