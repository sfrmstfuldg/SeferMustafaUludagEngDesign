#!/usr/bin/env python

import cv2 as cv
import numpy as np
import equations as eq
import computeControl

class Aruco(object):
	'''
	This class detects the ArUCo markes in the given image using OpenCV. Shows the
	markers in the image and calculates the area and the distance of the marker from camera frame
	center.
	'''

	def __init__(self,dictionary):
		
		self.ARUCO_DICT={
		"DICT_4X4_50": cv.aruco.DICT_4X4_50,
		"DICT_4X4_100": cv.aruco.DICT_4X4_100,
		"DICT_4X4_250": cv.aruco.DICT_4X4_250,
		"DICT_4X4_1000": cv.aruco.DICT_4X4_1000,
		"DICT_5X5_50": cv.aruco.DICT_5X5_50,
		"DICT_5X5_100": cv.aruco.DICT_5X5_100,
		"DICT_5X5_250": cv.aruco.DICT_5X5_250,
		"DICT_5X5_1000": cv.aruco.DICT_5X5_1000,
		"DICT_6X6_50": cv.aruco.DICT_6X6_50,
		"DICT_6X6_100": cv.aruco.DICT_6X6_100,
		"DICT_6X6_250": cv.aruco.DICT_6X6_250,
		"DICT_6X6_1000": cv.aruco.DICT_6X6_1000,
		"DICT_7X7_50": cv.aruco.DICT_7X7_50,
		"DICT_7X7_100": cv.aruco.DICT_7X7_100,
		"DICT_7X7_250": cv.aruco.DICT_7X7_250,
		"DICT_7X7_1000": cv.aruco.DICT_7X7_1000,
		"DICT_ARUCO_ORIGINAL": cv.aruco.DICT_ARUCO_ORIGINAL
		}
		self.arucoDict=cv.aruco.Dictionary_get(self.ARUCO_DICT[dictionary])

		self.MARKER_NAME_DICT={
		42:"CHARGER",
		87:"DROP",
		66:"PATIENT"
		}
		self.control = computeControl.Control(0.1)
		self.frame=0
		self.arucoParams = cv.aruco.DetectorParameters_create()
		self.frameCenter = (0,0)
		self.markerCenter = (0,0)
		self.Area = 0
		self.is_marker_detected = False
		self.markerTopCenter = (0,0) # to calculate psi in computeControl

	def look_at_the_marker(self,img):
		self.frame = img
		self.height,self.width,self.channels = self.frame.shape
		self.frameCenter = (int(self.width/2),int(self.height/2))
		# detect ArUCo markers in the input frame
		(corners,ids,rejected)=cv.aruco.detectMarkers(self.frame,self.arucoDict,parameters=self.arucoParams)

		# verify 'at least' one ArUCo marker was detected
		if len(corners)>0:
			self.is_marker_detected = True
			ids=ids.flatten()

			for (markerCorner, markerID) in zip(corners,ids):

				# extract the marker corners (which are always returned
				# in top-left, top-right, bottom-right, and bottom-left
				# order)
				corners = markerCorner.reshape((4, 2))
				(topLeft, topRight, bottomRight, bottomLeft) = corners

				# convert each of the (x, y)-coordinate pairs to integers
				TopRight = (int(topRight[0]), int(topRight[1]))
				BottomRight = (int(bottomRight[0]), int(bottomRight[1]))
				BottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
				TopLeft = (int(topLeft[0]), int(topLeft[1]))

				# draw the bounding box of the ArUCo detection
				cv.line(self.frame, TopLeft, TopRight, (0, 255, 0), 2)
				cv.line(self.frame, TopRight, BottomRight, (0, 255, 0), 2)
				cv.line(self.frame, BottomRight, BottomLeft, (0, 255, 0), 2)
				cv.line(self.frame, BottomLeft, TopLeft, (0, 255, 0), 2)

				# compute and draw the center (x, y)-coordinates of the
				# ArUco marker
				cX = int((topLeft[0] + bottomRight[0]) / 2.0)
				cY = int((topLeft[1] + bottomRight[1]) / 2.0)
				self.markerCenter = (cX,cY)

				cv.circle(self.frame, self.markerCenter, 4, (0, 0, 255), -1)

				#-------------------

				#Predicted frame positions

				#cv.circle(self.frame, (int(self.control.estimatedStatePose[0]),int(self.control.estimatedStatePose[1])), 4, (0, 0, 255), -1)
				#cv.circle(self.frame, (int(self.control.nextStatePose[0]),int(self.control.nextStatePose[1])), 4, (0, 0, 255), -1)

				#-------------------

				#draw the direction of the marker; from the marker center
				#to the middle top
				oX = int((topLeft[0] + topRight[0]) / 2.0)
				oY = int((topLeft[1] + topRight[1]) / 2.0)
				self.markerTopCenter = (oX,oY)
				#(oX, oY) center of the top corners

				cv.line(self.frame, self.markerCenter, self.markerTopCenter,(0, 0, 255), 2)  

				#Area of the marker
				self.Area = round(eq.distance(TopLeft,TopRight)*eq.distance(TopLeft,BottomLeft),2)

				#compute and draw the distance between the marker center and
				#the frame center. Change the color of the drawing reative to
				#distance
				dist = eq.distance(self.frameCenter,self.markerCenter)
		    
				if dist <25:
					cv.line(self.frame, self.markerCenter, self.frameCenter, (0, 255, 0), 2)
				elif 25< dist <75:
					cv.line(self.frame, self.markerCenter, self.frameCenter, (0, 255, 255), 2)
				else: 
					cv.line(self.frame, self.markerCenter, self.frameCenter, (0, 0, 255), 2)

				# draw the ArUco marker name on the frame w.r.t its ID 
				cv.putText(self.frame, self.MARKER_NAME_DICT[markerID],
					(TopLeft[0], TopLeft[1] - 15),
					cv.FONT_HERSHEY_SIMPLEX,
					0.5, (0,255,0), 2)      
		else:
			self.is_marker_detected = False
			self.markerCenter = (0,0)


	def get_frame(self):
		return self.frame

	def frameCenter(self):
		return self.frameCenter

	def markerCenter(self):
		return self.markerCenter

	def markerSurfaceArea(self):
		return self.Area

	def is_detected(self):
		return self.is_marker_detected

	def frameMarkerYawAngle(self):
		markerTopCenter = np.array(self.markerTopCenter)
		markerCenter = np.array(self.markerCenter)
		markerDirectionVector = np.array(markerTopCenter-markerCenter)
		frameDirectionVector = np.array((0,-self.frameCenter[1]))

		psi = round((np.arcsin(((np.cross(markerDirectionVector,frameDirectionVector))/(eq.distance(np.zeros((2,1)),markerDirectionVector)*eq.distance(np.zeros((2,1)),frameDirectionVector))))),2)
		
		return psi
