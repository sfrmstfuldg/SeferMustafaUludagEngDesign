#!/usr/bin/env python

import cv2 as cv
import rospy
import time
import sensor 
import aruco
import navControls
import computeControl
import equations as eq
import numpy as np

class Main(object):
	def __init__(self):
		self.dt=0.1

		self.data = sensor.SensorData()
		self.detect = aruco.Aruco("DICT_5X5_100")
		self.nav = navControls.Navigation()
		self.con = computeControl.Control(self.dt)

		self.prevTime = 0
		self.data.init_node()
		rospy.loginfo("emmi koddayiz")
		self.is_steady_state = False
		self.is_landed = False

		self.MARKER_REMINDER_DICT = {}

		
		self.r = rospy.Rate(10)

	def loop(self):
		while not rospy.is_shutdown():
			
			img=self.data.get_image()
			self.detect.look_at_the_marker(img)
			frame = self.detect.get_frame()
			
			yaw = self.nav.get_yaw()
			alt=self.nav.altitude()

			u = self.con.image_error_control(self.dt,self.detect.markerCenter,
					self.detect.frameCenter, self.detect.Area, self.detect.Area,self.detect.frameMarkerYawAngle())

			control = np.around(np.dot(self.con.rotationMatrix(yaw),u.T),3)

			print("")
			print("")
			print("---------- INFO ----------")
			print("frame center: "+str(self.detect.frameCenter))
			print("marker center: "+str(self.detect.markerCenter))
			#print("estimated marker center: "+str(self.con.estimatedStatePose))
			#print("predicted marker center: "+str(self.con.nextStatePose))
			print("marker surface area: "+str(self.detect.Area))
			print("marker yaw relative to frame: "+str(self.detect.frameMarkerYawAngle()))
			print("is marker detected: "+str(self.detect.is_marker_detected))
			print("yaw: "+str(yaw))
			print("altitude: "+str(alt))
			print("saved marker pose: "+str(self.MARKER_REMINDER_DICT))
			print("is marker position in steady state: "+str(self.is_steady_state))
			print("---------- COTROL COMMAND ----------")
			print("control command: "+str(control))
			print("--------------------")
			print("")
			print("")

			# if able to lan: land
			# elif marker still detectign: go to steady state
			# elif marker seen once than disapper for a moment: use dictionaried marker pose to go steady state 
			# else never seen a marker: search for it

			if (self.is_steady_state and bool(self.MARKER_REMINDER_DICT)) or (alt >= -0.8):
				print("CONDITION I")
				u = self.con.image_error_control(self.dt,self.MARKER_REMINDER_DICT["pose"],
					self.detect.frameCenter, self.detect.Area, self.detect.Area,self.detect.frameMarkerYawAngle())

				if alt >= -0.7:
					print("CONDITION I.I")
					self.nav.set_speed_vx_vy_vz_wpsi(0,0,.1,0)
					self.MARKER_REMINDER_DICT={"pose":self.detect.markerCenter}

					if alt >=-0.4:
						print("CONDITION I.I.I")
						self.nav.set_speed_vx_vy_vz_wpsi(0,0,0,0)
						self.nav.land()
						self.is_landed = True
				else:
					print("CONDITION I.II")
					control = np.around(np.dot(self.con.rotationMatrix(yaw),u.T),3)
					self.nav.set_speed_vx_vy_vz_wpsi(eq.speed_bound(control[0]),-eq.speed_bound(control[1]), .1,eq.speed_bound(control[3]))
					self.MARKER_REMINDER_DICT={"pose":self.detect.markerCenter}

			elif self.detect.is_detected():
				print("CONDITION II")
				self.nav.set_speed_vx_vy_vz_wpsi(eq.speed_bound(control[0]),-eq.speed_bound(control[1]),
												(eq.speed_bound(control[2])),eq.speed_bound(control[3]))
				self.MARKER_REMINDER_DICT={"pose":self.detect.markerCenter}

			elif bool(self.MARKER_REMINDER_DICT) and 0:
				print("CONDITION III")
				u = self.con.image_error_control(self.dt,self.MARKER_REMINDER_DICT["pose"],
					self.detect.frameCenter, self.detect.Area, self.detect.Area,self.detect.frameMarkerYawAngle())

				control = np.around(np.dot(self.con.rotationMatrix(yaw),u.T),3)

				self.nav.set_speed_vx_vy_vz_wpsi(eq.speed_bound(control[0]),-eq.speed_bound(control[1]),
												(eq.speed_bound(control[2])),eq.speed_bound(control[3]))
			else:
				print("CONDITION IV")
				rospy.loginfo("ArUCo is not detected !")
				rospy.loginfo("Searching for ArUCo")
				self.nav.set_speed_vx_vy_vz_wpsi(0.1,0,0,0)

			self.is_steady_state= eq.is_steady_state(self.detect.markerCenter[0],self.detect.markerCenter[1],self.detect.frameMarkerYawAngle(),
													self.detect.frameCenter[0],self.detect.frameCenter[1],0,2)

			cv.imshow("Frame", frame)
			self.r.sleep()

			currentTime = time.time()
			self.dt=currentTime - self.prevTime
			self.prevTime = currentTime
			
			if cv.waitKey(1) & 0xFF == ord('q'):
				break
			elif self.is_landed:
				print("!!! Quadrotor Landed on the Marker !!!")
				break


def main():
	file = Main()

	file.nav.takeoff(2)
	if file.nav.altitude()>-1.7:
		while file.nav.altitude()>-1.7:
			file.nav.set_speed_vx_vy_vz_wpsi(0,0,-.2,0)
			print("Waiting to reach altitude: "+str(file.nav.altitude()))
			if file.nav.altitude()<-1.7:
				break

	file.loop()

if __name__ == '__main__':
	try:
		main()
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		exit()
