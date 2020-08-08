'''
Copyright (c) 2020 Hao Da (Kevin) Dong
@file       recordVideo.py
@date       2020/04/14
@brief      Simple script to record and save a video
@license    This project is released under the BSD-3-Clause license.
'''

import numpy as np
import cv2
import datetime

# Set desired video resolution, framerate and logging offset
resolution = (1280,720)
fps = 30
logOffset = 3 		# Skip this number of datapoints while logging

# Get datetime string to use as file name
dateString = datetime.datetime.now().strftime("%Y-%m-%d-%H_%M_%S")

# Create MP4 video file at 720p 30fps
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(dateString + '.mp4', fourcc, fps, resolution)

# Create video stream object
videoCapture = cv2.VideoCapture(0)

while(videoCapture.isOpened()):
	# Grab the current frame
	ret, image = videoCapture.read()

	# Display the frame
	cv2.imshow("Frame", cv2.resize(image, resolution))

	# Write frame to video file
	out.write(cv2.resize(image, resolution))

	# Exit if the user presses 'q'
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# Release video and file object handles
videoCapture.release()
out.release()
print('Video and file handles closed')
