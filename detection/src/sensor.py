#!/usr/bin/env python

import rospy
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates

class SensorData(object):

	def __init__(self):
		
		self.image_sub = rospy.Subscriber("/webcam/image_raw",Image,self.__camera_callback)
		self.odom_sub = rospy.Subscriber("/mavros/local_position/odom",Odometry,self.__odom_callback)
		self.state_sub = rospy.Subscriber("/gazebo/model_states",ModelStates,self.__state_callback)
		self.bridge_object = CvBridge()
		self.imgmsg = Image()
		self.odomdata = Odometry()
		self.statedata = ModelStates()

	def __camera_callback(self,img):
		self.imgmsg = img

	def __odom_callback(self,odom):
		self.odomdata=odom

	def __state_callback(self,state):
		self.statedata = state

	def get_image(self):
		image = self.bridge_object.imgmsg_to_cv2(self.imgmsg, desired_encoding="bgr8")
		return image

	def init_node(self):
		rospy.init_node('DroneNode',anonymous=True)
		print("node initialized")
