#!/usr/bin/env python3
"""
Script for Exercise 2
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
from math import sin,cos,atan2,sqrt,fabs,pi


length_rigid_seg1 = 1  # in m
length_rigid_seg2 = 1  # in m

class RrbotJointTorquesPublisher(Node):
	def __init__(self):
        	super().__init__('rrbot_joint_torques_publisher')
        	self.sub1 = self.create_subscription(JointState,'/joint_states',self.joint_callback,10)
        	self.sub1  # prevent unused variable warning
        
        	#Define publishers for each joint torque controller commands.
        	self.pub1 = self.create_publisher(Float64MultiArray,'/effort_controllers/commands', 10)
        	

        	self.torque1 = float(input("Enter the desired torque for base_mid (red_green) joint in Nm : "))
        	self.torque2 = float(input("Enter the desired torque for mid_top (green_blue) joint in Nm : "))
		
	def joint_callback(self, message):
        	
        	msg = Float64MultiArray()
        	msg.data = [2.5*self.torque1,1.2*self.torque2]
        	if abs(message.position[0])< 3.0 and abs(message.position[1])< 3.0  :
      		  	self.pub1.publish(msg)
      		  	self.get_logger().info('Publishing: "%s"' % msg.data)
        	else:
      		  	self.torque1 = 0.0
      		  	self.torque2 = 0.0
      		  	self.pub1.publish(msg)
      		  	print("Stopping the effort on joints after reaching the set limits of joint angles")
      		  	self.get_logger().info('Publishing: "%s"' % msg.data)

        	joint_base_mid_angle = message.position[0]
        	joint_mid_top_angle = message.position[1]

        	joint_base_mid_vel = message.velocity[0]
        	joint_mid_top_vel = message.velocity[1]
        	
        	q_y = (length_rigid_seg1*cos(joint_base_mid_angle)) + (length_rigid_seg2*cos((joint_base_mid_angle+joint_mid_top_angle)))
        	q_x = (length_rigid_seg1*sin(joint_base_mid_angle)) + (length_rigid_seg2*sin((joint_base_mid_angle+joint_mid_top_angle)))
        	print ("The joint angles for the base_mid and mid_top joints are:", joint_base_mid_angle, joint_mid_top_angle, "respectively" )
        	

        	print ("The joint velocities for the base_mid and mid_top joints are:", joint_base_mid_vel, joint_mid_top_vel, "respectively")
        	
        	print ("The cartesian coordinates of the end effector are:", q_x, q_y, "respectively" )
        	sleep(0.1)
	
	
def main(args = None):
	rclpy.init(args=args)

	rrbot_joint_torques_publisher = RrbotJointTorquesPublisher()

	rclpy.spin(rrbot_joint_torques_publisher)

    	# Destroy the node explicitly
    	# (optional - otherwise it will be done automatically
    	# when the garbage collector destroys the node object)
	rrbot_joint_torques_publisher.destroy_node()
	rclpy.shutdown()




#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
	try: 
		main()
	except rospy.ROSInterruptException: 
		pass
	rclpy.spin()
