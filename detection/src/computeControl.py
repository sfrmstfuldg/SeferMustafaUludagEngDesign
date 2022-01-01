#!/usr/bin/env python

import numpy as np
import equations as eq
import sensor
from time import sleep



class Control(object):
	def __init__(self,dt):
		self.dt = dt

		self.Kp = 0.001
		self.Kd = 0.00045
		self.Ki = 0
		self.PID = np.array([self.Kp,self.Kd,self.Ki])
		self.error_prev = np.zeros(4)
		
		self.g = 0.00005
		self.h = 0.003
		self.k = 0.001
		self.GHK=np.array([self.g,self.h,self.k])

		self.initialGuessPose = np.array([413,210]) #(x,y)
		self.initialGuessVelocity = np.array([1,1])*10**-9 #(vx,vy)
		self.initialGuessAccel = np.array([0,0]) #(ax,ay)
		self.initialGuessState =np.array([self.initialGuessPose,self.initialGuessVelocity,self.initialGuessAccel])
		self.prevState = eq.stateExtrapolation2D(self.initialGuessState,self.dt)
		self.estimatedStatePose = np.zeros(2)
		self.nextStatePose = np.zeros(2)



	def image_error_control(self,dt,p1,p2,A,desiredA,yaw):
		'''
		:param dt:	time differet
		:param p1:	MEASURED marker center (x,y)
		:param p2:	DESIRED frame center
		:param A:	MEASURED area of the marker
		:param desiredA: DESIRED area of the marker
		:param yaw:	angle between marker and the frame
		:returns: 	error matrix to compute control commands
		'''
		
		markerCenter = np.array(p1)
		frameCenter = np.array(p2)

		'''
		#START: MARKER CENTER PREDICTION

		#---------- NOT WORKING PROPERLY ----------#

		measuredPose = markerCenter
		
		estimated_state = eq.stateUpdate2D(self.prevState,measuredPose,self.GHK,dt)
		print("estimated_state_x: "+str(int(estimated_state[0][0])))

		next_state = eq.stateExtrapolation2D(estimated_state,dt)
		print("predicted_state: "+str(np.around(next_state[0],2)))
		
		self.estimatedStatePose = estimated_state[0]
		self.nextStatePose = next_state[0]
		self.prevState = next_state

		#END: MARKER CENTER PREDICTION
		'''

		desired_x_y_area_psi = np.vstack((frameCenter,[np.sqrt(desiredA),0])).flatten()
		measured_x_y_area_psi = np.vstack((markerCenter,[np.sqrt(A),yaw])).flatten()
		print("desired pose: "+str(desired_x_y_area_psi))
		print("measured pose: "+str(measured_x_y_area_psi))

		error_p = desired_x_y_area_psi - measured_x_y_area_psi
		error_d = (error_p-self.error_prev)/dt
		error_i = error_p + self.error_prev
		error_prev=error_p

		error_matrix = np.vstack((error_p,error_d,error_i)).T
		image_error_control = np.dot(error_matrix,self.PID.T) # [Vy, Vx, Vz, Wpsi]

		xyzw = np.array([image_error_control[1],image_error_control[0],image_error_control[2],image_error_control[3]])

		return xyzw

	def rotationMatrix(self,yaw):

		mat = np.array([[np.cos(yaw),	np.sin(yaw),	0,	0],
						[-np.sin(yaw),	np.cos(yaw),	0,	0],
						[0,				0,				5,	0],
						[0,				0,				0,	100]])

		return mat



