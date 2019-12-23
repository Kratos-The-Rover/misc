#! /usr/bin/env python

#                                                       #
#                                                       #
#		ROTATE COMPLETELY AND THEN MOVE FORWARD.        #
#														#
#														#


import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import *
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
import math
import tf
import time
import numpy as np

import sys


MAX_ANGULAR_VEL = 2.84
MAX_TURNING_VEL=1.5
STOP_DIST = 0.2


class husky_controller():
	def __init__(self,c):
		self.track_list = c
		self.track_ind = 0

		self.pose = Twist()
		self.odom = Odometry()
		self.curr_pos = np.array([0,0])

		self.flag=0
		self.last_yaw=0
		self.last_x=0
		self.last_y=0

		self.vel_pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist,queue_size=10)
		self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_cb)

		self.rate = rospy.Rate(100)
		self.rate2 = rospy.Rate(50)

	def planner(self):
		
		while 1:
			current_yaw=self.pose.angular.z
			
			dest_yaw = np.math.atan2(self.track_list[-1][1],self.track_list[-1][0])

			print("difference:")
			# print(self.pose.linear.x - self.track_list[-1][0])
			print(self.curr_pos)
			print(self.curr_pos - self.track_list[-1])


			if((abs(self.pose.linear.x - self.track_list[-1][0]) < STOP_DIST) and (abs(self.pose.linear.y - self.track_list[-1][1]) < STOP_DIST)):
				print ("CONDITION SATISFIED:: ROVER STOPPED")
				twist_obj = Twist()
				self.flag = 0
				self.vel_pub.publish(twist_obj)
				# break	

			if((abs(current_yaw-dest_yaw) > 0.1)) and not self.flag:
				twist_obj=Twist()
				twist_obj.angular.z = (kp*(dest_yaw-current_yaw) + kd*(current_yaw-self.last_yaw))
				if(twist_obj.angular.z>MAX_TURNING_VEL):
					twist_obj.angular.z=MAX_TURNING_VEL
				if(twist_obj.angular.z<-MAX_TURNING_VEL):
					twist_obj.angular.z=-MAX_TURNING_VEL
				
				# if (twist_obj.angular.z < 0):
				# 	twist_obj.angular.z=-MAX_TURNING_VEL
				# else:
				# 	twist_obj.angular.z= MAX_TURNING_VEL

				# print (twist_obj.angular.z, "   TURNING")

	
				self.vel_pub.publish(twist_obj)
				self.last_yaw = current_yaw
				self.last_x = self.pose.linear.x
				self.last_y = self.pose.linear.y

			elif (((abs(current_yaw-dest_yaw) < 0.1)) and not self.flag):
				self.flag=1
				print ("TURNING COMPLETE ",self.flag)
				twist_obj=Twist()
				twist_obj.angular.z = 0
				twist_obj.linear.x = 0
				self.vel_pub.publish(twist_obj)

				print(self.track_list[-1])


			if(((abs(self.pose.linear.x-self.track_list[-1][0])>0.02) or (abs(self.pose.linear.y-self.track_list[-1][1])>0.02)) and self.flag==1 ):
				twist_obj=Twist()
				twist_obj.linear.x=fwd_vel
				twist_obj.angular.z = klx * (-self.track_list[self.track_ind][0] + self.curr_pos[0]) + kly * (self.track_list[self.track_ind][1] - self.curr_pos[1]) 
				#print(twist_obj.angular.z) 
				if(twist_obj.angular.z>MAX_TURNING_VEL):
					twist_obj.angular.z=MAX_TURNING_VEL
				if(twist_obj.angular.z<-MAX_TURNING_VEL):
					twist_obj.angular.z=-MAX_TURNING_VEL

				# print twist_obj.angular.z, "     MOVING FORWARD"
				###########################################
				# print("MOVING FORWARD(linear_x)->")
				# print(self.pose.linear.x, self.pose.linear.y)
				
				# print("MOVING FORWARD(angular_z)->")
				# print(twist_obj.angular.z)
				###########################################
	
				self.vel_pub.publish(twist_obj)
				self.last_yaw=current_yaw
				self.last_x=self.pose.linear.x
				self.last_y=self.pose.linear.y
			
			self.rate2.sleep()

		sys.exit()
				

	def get_nearest_ind(self):
		d = np.array([np.sqrt(np.sum((self.curr_pos - i)**2)) for i in self.track_list])
		self.track_ind = np.argmin(d)


	def odom_cb(self,data):
		# print(data)
		self.odom = data
		self.pose.linear.x = data.pose.pose.position.x
		self.pose.linear.y = data.pose.pose.position.y
		self.pose.linear.z = data.pose.pose.position.z

		quaternion = (data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)

		self.pose.angular.x = euler[0]
		self.pose.angular.y = euler[1]
		self.pose.angular.z = euler[2]

		self.get_nearest_ind()
		self.curr_pos = self.pose_to_node()
		# self.rate.sleep()


	def pose_to_node(self):
		return np.array([self.pose.linear.x , self.pose.linear.y])


if __name__=="__main__":
	try:
		rospy.init_node("husky_controller")
		
		start = np.array([ 10.0886432687 , 10.1030427763])
		end = np.array([0.0 , 0.0])
		step_size = 0.05

		dv = end - start
		dv = dv / np.sqrt(np.sum(dv**2))
		
		c = np.array([start])
		i=1
		st_goal_dist = np.sqrt(np.sum((end - start)**2))
		while np.sqrt(np.sum((c[i-1] - start)**2)) < st_goal_dist:    #( (abs(c[i-1][0]) - abs(end[0])) > 0 or (abs(c[i-1][1]) - abs(end[1])) > 0):
			new_pt = start + i * step_size * dv
			# print(new_pt)
			c=np.concatenate((c,np.array([new_pt])),axis=0)
			i += 1

		###########################################
		# print("c->")
		# print(c[-5:])
		###########################################
		###########################################
		# print("ST_GOAL , LAST_GOAL->")
		# print(st_goal_dist , np.sqrt(np.sum(c[-1] - start)**2))
		###########################################
		# plt.figure()
		# plt.plot((start[0],end[0]),(start[1],end[1]),'r-')
		# a = 
		# print(a)
		# plt.plot([x[0] for x in c] , [y[1] for y in c],'g*')
		# plt.show()
		c[-1]=end

		###########################################
		# print("c->")
		# print(c)
		###########################################

		my_controller = husky_controller(c)
		

		fwd_vel=1
		kp = 0.5
		kd = 40
		klx = 0.8
		kly = 0.8

		if end[1]-start[1] < 0:
			klx = -1 * klx
		if end[0]-start[0] < 0:
			kly = -1 * kly
		# for pt in range(len(c_x)):
		my_controller.planner()
		
		rospy.spin()
	except Exception as e:
		print("exception",e)