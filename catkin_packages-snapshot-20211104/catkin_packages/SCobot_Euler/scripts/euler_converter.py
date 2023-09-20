#!/usr/bin/env python
import rospy
import numpy as np
import numpy.linalg as la
import numpy.random as rnd
import matplotlib.pyplot as plt
import tf


from std_msgs.msg import Int32
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

class Converter:
	def __init__(self):
		self.euler_pub=rospy.Publisher('/scobot/euler',Vector3, queue_size=0)
		rospy.Subscriber("/scobot/imu", Imu, self.callback)


	def callback(self,data):
		quat = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)

		euler = tf.transformations.euler_from_quaternion(quat)
		ros_msg = Vector3()
		ros_msg.x = euler[0]
		ros_msg.y = euler[1]
		ros_msg.z = euler[2]
		self.euler_pub.publish(ros_msg)


	def listener(self):
		rospy.Subscriber("/scobot/imu", Imu, self.callback)
		rospy.spin()


if __name__=='__main__':
	rospy.init_node('euler_converter')
	converter = Converter()
	rospy.spin()
	rospy.loginfo('END')
