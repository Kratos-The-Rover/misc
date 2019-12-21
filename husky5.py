#! /usr/bin/env python
#                                                       #
#                                                       #
#		ROTATE COMPLETELY AND THEN MOVE FORWARD.        #
#														#
#														#
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
import math
import tf
import time
import numpy as np



MAX_ANGULAR_VEL = 2

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
		rate=rospy.Rate(100)

	def planner(self):
		
		while 1:

			current_yaw=self.pose.angular.z
			#dest_yaw = math.acos(self.track_list[-1][0]/(math.sqrt(self.track_list[-1][0]**2+self.track_list[-1][1]**2)))
			dest_yaw=np.math.atan2(self.track_list[-1][1],self.track_list[-1][0])
			if((abs(self.pose.linear.x - self.track_list[-1][0])<0.02) and (abs(self.pose.linear.y-self.track_list[-1][1])<0.02)):
				print "CONDITION SATISFIED:: ROVER STOPPED"
				twist_obj=Twist()
				self.flag=0
				self.vel_pub.publish(twist_obj)
				break	

			if((abs(current_yaw-dest_yaw)>0.01)) and not self.flag:
				twist_obj=Twist()
				twist_obj.angular.z = (kp*(dest_yaw-current_yaw) + kd*(current_yaw-self.last_yaw))
				# print twist_obj.angular.z, "   TURNING", 
				self.vel_pub.publish(twist_obj)
				self.last_yaw=current_yaw
				self.last_x=self.pose.linear.x
				self.last_y=self.pose.linear.y

			else:
				self.flag=1
				print "TURNING COMPLETE    ",self.flag
				# break

			if(((abs(self.pose.linear.x-self.track_list[-1][0])>0.3) or (abs(self.pose.linear.y-self.track_list[-1][1])>0.3)) and self.flag==1 ):
				twist_obj=Twist()
				twist_obj.linear.x=fwd_vel
				twist_obj.angular.z = -kl * (self.track_list[self.track_ind][0] - self.curr_pos[0]) 

				self.vel_pub.publish(twist_obj)
				self.last_yaw=current_yaw
				self.last_x=self.pose.linear.x
				self.last_y=self.pose.linear.y
				

	def get_nearest_ind(self):
		d = np.array([np.sqrt(np.sum((self.curr_pos - i)**2)) for i in self.track_list])
		self.track_ind = np.argmin(d)


	def odom_cb(self,data):
		self.odom = data
		self.pose.linear.x = data.pose.pose.position.x
		self.pose.linear.y = data.pose.pose.position.y
		self.pose.linear.z = data.pose.pose.position.z

		quaternion = (data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
		euler=tf.transformations.euler_from_quaternion(quaternion)

		self.pose.angular.x = euler[0]
		self.pose.angular.y = euler[1]
		self.pose.angular.z = euler[2]

		self.get_nearest_ind()
		self.curr_pos = self.pose_to_node()



	def pose_to_node(self):
		return np.array([self.pose.linear.x , self.pose.linear.y])

		



if __name__=="__main__":
	rospy.init_node("husky_controller")
	start = np.array([0,0])
	end = np.array([-10,-
		5])
	step_size = 0.1

	dv = end - start
	dv = dv / np.sqrt(np.sum(dv**2))
	
	c = np.array([start])
	i=1
	# print(dv)
	while (abs(c[i-1][0])<abs(end[0]) and abs(c[i-1][1])<abs(end[1])):
		new_pt = start + i * step_size * dv
		print(new_pt)
		c=np.concatenate((c,np.array([new_pt])),axis=0)
		i += 1

	c[-1]=end
	print(c)
	my_controller = husky_controller(c)
	

	fwd_vel=0.4
	kp = 0.5
	kd = 40
	kl = 0.2
	# kp1=(-0.5)
	# kp2=(0.5)		
	# kd1= -6000
	# kd2= 6000
	# in_x=0
	# in_y=0	
	# dest_x = 5
	# dest_y = 20
	# step_size=0.1
	# c_x[0]=0
	# c_y[0]=0
	

	# for pt in range(len(c_x)):
	my_controller.planner()


	rospy.spin()

