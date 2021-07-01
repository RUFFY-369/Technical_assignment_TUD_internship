#!/usr/bin/env python3

"""
Script for Exercise 3
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import math
import time
from time import time
from time import sleep


from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from math import sin,cos,atan2,sqrt,fabs,pi,acos,asin, pow

length_rigid_seg1 = 1  # in m
length_rigid_seg2 = 1  # in m


class InverseKinematicsAndJointPublisher(Node):

	def __init__(self):
        	super().__init__('inverse_kinematics_and_joint_publisher')
        	#Define publishers for each joint position controller commands.
        	self.pub1 = self.create_publisher(Float64MultiArray,'/position_controllers/commands', 10)
        	timer_period = 0.5 #(seconds)
        	self.timer = self.create_timer(timer_period, self.timer_callback)

		
	def timer_callback(self):
        	#Analytical approach for Inverse Kinematics of the Robotic Arm

        	desired_cart_x = float(input("Enter your desired y cartesian coordinate (Green Line as seen) value of end effector w.r.t base (red link) with gazebo world frame (Right Hand   						Coordinate system) as reference: "))
        	desired_cart_y = float(input("Enter your  desired x cartesian coordinate (Red Line as seen) value of end effector w.r.t base (red link) with gazebo world frame (Right Hand 						Coordinate system) as reference: "))

        	joint_mid_top_angle = acos((pow(desired_cart_x,2)+ pow(desired_cart_y,2) - pow(length_rigid_seg1,2) - pow(length_rigid_seg2,2))/(2*length_rigid_seg1*length_rigid_seg2))
        
        	psi = atan2((length_rigid_seg1 + (length_rigid_seg2*cos(joint_mid_top_angle))),(length_rigid_seg2*sin(joint_mid_top_angle)))

        	joint_base_mid_angle = psi - atan2(desired_cart_x,desired_cart_y)

        	msg = Float64MultiArray()
        	msg.data = [joint_base_mid_angle,joint_mid_top_angle]
		#Publish the required configuration to each joint.
        	self.pub1.publish(msg)
        	self.get_logger().info('The joint angles for the base_mid and mid_top joints are: "%s"' % msg.data)

		# TWO POSSIBLE SOLUTIONS
		# msg.data = [-joint_base_mid_angle,-joint_mid_top_angle]
		# self.pub1.publish(msg)



def main(args = None):
	rclpy.init(args=args)

	inverse_kinematics_and_joint_publisher = InverseKinematicsAndJointPublisher()

	rclpy.spin(inverse_kinematics_and_joint_publisher)

    	# Destroy the node explicitly
    	# (optional - otherwise it will be done automatically
    	# when the garbage collector destroys the node object)
	inverse_kinematics_and_joint_publisher.destroy_node()
	rclpy.shutdown()



#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
	try: 
        	main()
	except rospy.ROSInterruptException: 
        	pass
	rclpy.spin()
