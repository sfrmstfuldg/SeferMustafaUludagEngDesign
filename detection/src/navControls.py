#!/usr/bin/env python

from pymavlink import mavutil
from time import sleep

class Navigation(object):

	def __init__(self):
		# Start a connection listening to a UDP port
		self.the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

		# Wait for the first heartbeat 
		#   This sets the system and component ID of remote system for the link
		self.the_connection.wait_heartbeat()
		print("Heartbeat from system (system %u component %u)" % 
				(self.the_connection.target_system, self.the_connection.target_component))

	def arm_disarm(self,key):
		'''
		Arms or disarms the quadrotor w.r.t key value
		:param key: int number 1 or 0
		:return arm if key == 1 ; disarm if key == 0 ; error if not key == 1 or 0
		'''
		if (key == 1):
			self.self.the_connection.mav.command_long_send(self.self.the_connection.target_system, self.the_connection.target_component,
						mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,key,0,0,0,0,0,0)

			msg = self.self.the_connection.recv_match(type='COMMAND_ACK',blocking=True)
			print(msg)
		elif (key == 0):
			self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component,
						mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,key,0,0,0,0,0,0)

			msg = self.the_connection.recv_match(type='COMMAND_ACK',blocking=True)
			print(msg)
		else:
			print("Key is not invalid. Must be 1 to arm, 0 to disarm")

	def takeoff(self,height):
		self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component,
	                                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0)
		msg = self.the_connection.recv_match(type='COMMAND_ACK',blocking=True)
		print(msg)
		sleep(2)
		self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component,
		                                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,height)
		msg = self.the_connection.recv_match(type='COMMAND_ACK',blocking=True)
		print(msg)

	def land(self):
		self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component,
	                                    mavutil.mavlink.MAV_CMD_NAV_LAND,0,0,0,0,0,0,0,0)
		msg = self.the_connection.recv_match(type='COMMAND_ACK',blocking=True)
		print(msg)

	def return_to_launch(self):
		self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component,
	                                    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,0,0,0,0,0,0,0,0)
		msg = self.the_connection.recv_match(type='COMMAND_ACK',blocking=True)
		print(msg)

	def set_speed_vx_vy_vz_wpsi(self,vx,vy,vz,wpsi):
		'''
		Sets the linear and angular speed of the quadrotor in local referance frame
		:param vx: linear speed in x axis of the quadrotor [m/s]
		:param vy: linear speed in y axis of the quadrotor [m/s]
		:param vz: linear speed in z axis of the quadrotor [m/s]
		:param wpsi: angular speed in yaw angle of the quadrotor [rad/s] 
		'''
		#typmask : int(0b(yawrate)(yaw)0(az)(ay)(ax)(vz)(vy)(vx)(z)(y)(x)); 0 => control; 1=> not control
		self.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,
							self.the_connection.target_system, self.the_connection.target_component,
							mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b010111000111), 0, 0, 0,
							vx, vy, vz, 0, 0, 0, 0, wpsi))

	def set_position_local(self,x,y,z,psi):
		'''
		Sets the linear and angular speed of the quadrotor in local referance frame
		:param x: position in x axis relative to quadrotors frame [m]
		:param y: position in y axis relative to quadrotors frame [m]
		:param z: position in z axis relative to quadrotors frame [m]
		:param yaw: angle in yaw axis relative to quadrotors frame [rad] 
		'''
		#typmask : int(0b(yawrate)(yaw)0(az)(ay)(ax)(vz)(vy)(vx)(z)(y)(x)); 0 => control; 1=> not control
		self.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,
							self.the_connection.target_system, self.the_connection.target_component,
							mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b100111111000), x, y, z,
							0, 0, 0, 0, 0, 0, psi, 0))

	def get_yaw(self):

		msg = self.the_connection.recv_match(type='ATTITUDE',blocking=True)
		return msg.yaw

	def altitude(self):

		msg = self.the_connection.recv_match(type='LOCAL_POSITION_NED',blocking=True)
		return(msg.z)
