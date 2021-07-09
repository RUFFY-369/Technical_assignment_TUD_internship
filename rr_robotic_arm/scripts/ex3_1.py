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

errors = np.array([0.0,0.0])
del_error = np.array([0.0,0.0])
prev_error = np.array([0.0,0.0])
sum_error = np.array([0.0,0.0])
joint_base_mid_angle_state, joint_mid_top_angle_state = 0.0,0.0

def inverse_kinematics_and_joint_publisher():
	global errors, joint_base_mid_angle_state, joint_mid_top_angle_state, del_error, prev_error, sum_error, base_mid_angle_effort, mid_top_angle_effort

	limit_pwm = lambda num,max_num,min_num : max(min(num,max_num),min_num)

	#Initiate node for controlling joint1 and joint2 positions.
	rospy.init_node('rrbot_joint_positions_node', anonymous=True)

	#Define publishers for each joint position controller commands.
	
	pub3 = rospy.Publisher('/rrbot/joint_base_mid_torque_controller/command', Float64, queue_size=100)
	pub4 = rospy.Publisher('/rrbot/joint_mid_top_torque_controller/command', Float64, queue_size=100)
	
	rospy.Subscriber('/rrbot/joint_states', JointState, callback)

	# Tuned PID Gains
	Kp = [10.0, 10.0]
	Kd = [55.0, 55.0]
	Ki = [0.0005,0.0005]

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

		errors[0] = joint_base_mid_angle - joint_base_mid_angle_state
		errors[1]  = joint_mid_top_angle - joint_mid_top_angle_state
        

		while abs(errors[0]) > 0.009 or abs(errors[1]) > 0.005 or abs(base_mid_angle_effort) > 3e-4 or abs(mid_top_angle_effort)>3e-4 :
			# print("in while", errors,base_mid_angle_effort,mid_top_angle_effort)
			errors[0] = joint_base_mid_angle - joint_base_mid_angle_state
			errors[1]  = joint_mid_top_angle - joint_mid_top_angle_state

			sum_error[0]+= errors[0]
			sum_error[1]+= errors[1]

			del_error[0] =  (errors[0] - prev_error[0])
			del_error[1] =  (errors[1] - prev_error[1])
            

			torque_joint_base_mid = limit_pwm((Kp[0] * errors[0] + Ki[0]*sum_error[0] + Kd[0] * del_error[0]), 2.5, -2.5)
			torque_joint_mid_top = limit_pwm((Kp[1] * errors[1] + Ki[1]*sum_error[1] + Kd[1] * del_error[1]), 2.5, -2.5)

			prev_error[0]= errors[0]
			prev_error[1]= errors[1]
			pub3.publish(torque_joint_base_mid)
			pub4.publish(torque_joint_mid_top)
			sleep(0.1)
        
		while (base_mid_angle_effort != 0 or mid_top_angle_effort!=0):
			#Publish the required efforts to each joint.
			pub3.publish(0.0)
			pub4.publish(0.0)

		# TWO POSSIBLE SOLUTIONS
		# pub1.publish(-joint_base_mid_angle)
		# pub2.publish(-joint_mid_top_angle)


		rate.sleep() #sleep for rest of rospy.Rate(100)

def callback(message):
	global joint_base_mid_angle_state, joint_mid_top_angle_state, base_mid_angle_effort, mid_top_angle_effort
	joint_base_mid_angle_state = message.position[0]
	joint_mid_top_angle_state = message.position[1]
	base_mid_angle_effort = message.effort[0]
	mid_top_angle_effort = message.effort[1]

#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
	try: 
		inverse_kinematics_and_joint_publisher()
	except rospy.ROSInterruptException: 
		pass
	rospy.spin()