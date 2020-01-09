#! /usr/bin/env python
import rospy
from sensor_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *

fwd_value = 0  #axes[1]
dir_value = 0 #axes[3]

mapped_fwd_vel = 0 #controls the forward and backward movement
mapped_dir_vel = 0 #controls the sideways movement

rospy.init_node('husky_controller')


def mymap(c,a,b,d,e):
	return d + (c-a)*(e-d)/(b-a);

def cb(joy_object):
	global fwd_value
	global dir_value
	global mapped_dir_vel
	global mapped_fwd_vel
	#Control algorithm and code
	fwd_value=joy_object.axes[1]
	dir_value=joy_object.axes[3]

	mapped_fwd_vel=mymap(fwd_value,1.0,-1.0,-6,6)
	if(-0.1<fwd_value<0.2):
		mapped_fwd_vel=0
	mapped_dir_vel=mymap(dir_value,1,-1,1,-1)
	if(-0.1<dir_value<0.2):
		mapped_dir_vel=0


pub_to_husky=rospy.Publisher('/husky_velocity_controller/cmd_vel',Twist) #publisher to husky
pub_to_cmdvel=rospy.Publisher('/cmd_vel',Twist) #publisher to cmd_vel

rate=rospy.Rate(10)



while not rospy.is_shutdown():
	sub_to_joy=rospy.Subscriber('joy', Joy, cb)

	twist_obj1=Twist()
	twist_obj1.linear.x=mapped_fwd_vel
	twist_obj1.angular.z=mapped_dir_vel

	pub_to_husky.publish(twist_obj1)
	pub_to_cmdvel.publish(twist_obj1)
	rate.sleep()
