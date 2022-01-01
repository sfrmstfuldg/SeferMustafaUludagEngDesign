#!/usr/bin/env python

import numpy as np

def speed_bound(speed):
	if speed > .5:
		speed=.5
	if speed < -.5:
		speed = -.5
	return speed

def distance(p1,p2):
	'''
	calculates the distance between two points

	:param p1: - point1 (x1,y1)
	:param p2: - point2 (x2,y2)
	:return distance
	'''

	dist= np.sqrt(np.power((p1[0]-p2[0]),2) + np.power((p1[1]-p2[1]),2))

	return dist

def stateExtrapolation2D(state,dt):
	'''
	Calculates the predicted states from given states

	:param state: - currnet position,current velocity,current accelration [[x,y],[vx,vy],[ax,ay]] 
	:param dt: - delta_t
	:returns predicted state [[x',y'],[vx',vy'],[ax',ay']]
	'''

	time_matrix = np.array([1,dt,0.5*(dt**2)]).T

	state_matrix = np.array([[np.reshape((state[0],	state[1],	state[2]),6)],
							[np.reshape((state[1],	state[2],	np.zeros(2)),6)],
							[np.reshape((state[2], np.zeros(2),	np.zeros(2)),6)]]).T

	extrapolated_state = np.dot(state_matrix,time_matrix)

	return np.reshape(extrapolated_state,(3,-1)) 

def stateUpdate2D(prevState,measuredPose,ghk,dt):

	error_factor = measuredPose-prevState[0]

	update_matrix = np.array([[np.reshape(prevState,6)],
								[np.reshape((error_factor,np.zeros(2),np.zeros(2)),6)],
								[np.reshape((np.zeros(2),error_factor,np.zeros(2)),6)],
								[np.reshape((np.zeros(2),np.zeros(2),error_factor),6)]]).T

	time_matrix = np.array([1,1,1/dt,1/(0.5*(dt**2))]).T

	ghk_matrix = np.array([[1,0,0,0],
							[0,ghk[0],0,0],
							[0,0,ghk[1],0],
							[0,0,0,ghk[2]]])

	consants = np.dot(ghk_matrix,time_matrix).T

	estimated_stade = np.dot(update_matrix,consants)

	return np.reshape(estimated_stade,(3,-1))

def is_steady_state(mx,my,mw,dx,dy,dw,percent):
	mes=np.array([mx,my,mw])
	des=np.array([dx,dy,dw])
	steadyStateTresholdVlaues = [des-(des*percent/100), des+(des*percent/100)]
	#print("steadyStateTresholdVlaues: "+str(steadyStateTresholdVlaues))

	if np.less_equal(steadyStateTresholdVlaues[0],mes)[0] and np.less_equal(steadyStateTresholdVlaues[0],mes)[1] and np.less_equal(steadyStateTresholdVlaues[0],mes)[2]:
		if np.less_equal(mes,steadyStateTresholdVlaues[1])[0] and np.less_equal(mes,steadyStateTresholdVlaues[1])[1] and np.less_equal(mes,steadyStateTresholdVlaues[1])[2]:
			return True
		else: return False
	else: return False
		 


