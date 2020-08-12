'''
Copyright (c) 2020 Hao Da (Kevin) Dong, Krithika Govindaraj
@file       retrieveblock.py
@date       2020/05/15
@brief      Autonomous object recognition and retrieval with Baron robot
@license    This project is released under the BSD-3-Clause license.
'''

import RPi.GPIO as gpio
import serial
import time
import numpy as np
import math
import cv2
import imutils
import pyzbar.pyzbar as pyzbar

'''
############################## FILE TO STORE COORDINATES ##################
f = open("demofile.txt", "a")

################################## TRAJECTORY ########################################
X = [0] # X coords
Y = [0]	# Y coords
# forwardCoord = "Y"  initially if we move forward we are moving in Y
# backCoord = "Y"
# rightCoord = "X"   # initially if we move right we are moving in X
# leftCoord = "X"	   # initially if we move left we are moving in X
currentAngle = 90      # facing north
'''

############################# HELPER FUNCTIONS #######################################
def dist2Ticks(dist):
	return int((20/(np.pi*0.065)) * dist)

def deg2Ticks(deg):
	return int((20/(np.pi*0.065)) * (0.075*np.deg2rad(deg)))

def ticks2dist(ticks):
	return int(((np.pi*0.065)/20) * ticks)

def getIMUAngle():
	global ser
	ser.reset_input_buffer()
	while (ser.in_waiting == 0):
		continue

	# Read serial stream
	line = ser.readline()
	#print(line)

	# Strip newline and return carriage from line
	line = line.rstrip().lstrip()

	# Convert line to string, strip non-numeric characters and convert to float
	line = str(line)
	line = line.strip("'").strip("b'")
	print(line)
	angle = float(line)

	return angle


def getObjectLocation(image):
	sensorSizeY = 2.76	    # mm
	sensorResY = 2464	    # px
	focalLength = 3.04	    # mm
	objectHeight = 3.175	# cm
	# Account for 2x2 binning if resolution is (1280, 720)
	# https://picamera.readthedocs.io/en/release-1.12/fov.html
	pixPerMil = (sensorResY/2) / sensorSizeY	# px/mm

	decodedObjects = pyzbar.decode(image)

	centerX = 0
	centerY = 0

	# Print results
	for obj in decodedObjects:
		print('Type : ', obj.type)
		print('Data : ', obj.data,'\n')
		
		# Points start from top-left and progress CCW
		points = obj.polygon

		# Number of corners
		n = len(points)

		# Get center of QR code
		xCoordinates = [points[i].x for i in range(n)]
		yCoordinates = [points[i].y for i in range(n)]

		centerX = int((max(xCoordinates) + min(xCoordinates)) / 2)
		centerY = int((max(yCoordinates) + min(yCoordinates)) / 2)

		# Calculate average pixel height of QR code
		h = ((points[1].y - points[0].y) + (points[2].y - points[3].y)) / 2
		print(h)

		# Calculate distance to QR code
		distance = objectHeight * focalLength * pixPerMil / h

		# Draw QR code boundary
		for i in range(0, n):
			print(points[i])
			cv2.line(image, points[i], points[(i+1) % n], color=(255,0,0), thickness=3)
		
		# Print distance on QR code in image
		cv2.putText(image, str(round(distance, 1)) + 'cm', (centerX,centerY), \
				cv2.FONT_HERSHEY_SIMPLEX, 1, \
				color=(0,0,255), thickness=3)

	cX_frame = int(1280/2) # verify if this needs switching
	cY_frame = int(720/2) # verify if this needs switching

	angle = (centerX - cX_frame) * 0.061
	distance = objectHeight * focalLength * pixPerMil / h

	return angle, distance, image

##################### DRIVE FUNCTIONS ###########################
## Stop
def stopDriving():
	# Set all motor driver pins low
	gpio.output(31, False)
	gpio.output(33, False)
	gpio.output(35, False)
	gpio.output(37, False)


### Directions
def driveForward(distance):
	global dutyCycle, diff, leftPWMPin, rightPWMPin

	ticks = dist2Ticks(distance)
	counterBR = 0
	counterFL = 0
	buttonBR = int(0)
	buttonFL = int(0)

	#Left wheels
	gpio.output(31, True)
	gpio.output(33, False)

	# Right Wheels
	gpio.output(35, False)
	gpio.output(37, True)

	while (counterBR < ticks or counterFL < ticks):
		# Move in the direction of the object
		if (counterBR > counterFL):
			leftPWMPin.ChangeDutyCycle(dutyCycle + diff)
			rightPWMPin.ChangeDutyCycle(dutyCycle - diff)
		elif (counterBR < counterFL):
			leftPWMPin.ChangeDutyCycle(dutyCycle - diff)
			rightPWMPin.ChangeDutyCycle(dutyCycle + diff)
		else:
			leftPWMPin.ChangeDutyCycle(dutyCycle)
			rightPWMPin.ChangeDutyCycle(dutyCycle)
		
		# Update encoder states
		if (gpio.input(12) != buttonBR):
			buttonBR = int(gpio.input(12)) #holds the state
			counterBR += 1

		if (gpio.input(7) != buttonFL):
			buttonFL = int(gpio.input(7)) #holds the state
			counterFL += 1
	
	stopDriving()


def driveBackward(distance):
	global dutyCycle, diff, leftPWMPin, rightPWMPin

	ticks = dist2Ticks(distance)
	counterBR = 0
	counterFL = 0
	buttonBR = int(0)
	buttonFL = int(0)

	#Left wheels
	gpio.output(31, False)
	gpio.output(33, True)

	# Right Wheels
	gpio.output(35, True)
	gpio.output(37, False)	

	while (counterBR < ticks or counterFL < ticks):
		# Move in the direction of the object
		if (counterBR > counterFL):
			leftPWMPin.ChangeDutyCycle(dutyCycle + diff)
			rightPWMPin.ChangeDutyCycle(dutyCycle - diff)
		elif (counterBR < counterFL):
			leftPWMPin.ChangeDutyCycle(dutyCycle - diff)
			rightPWMPin.ChangeDutyCycle(dutyCycle + diff)
		else:
			leftPWMPin.ChangeDutyCycle(dutyCycle)
			rightPWMPin.ChangeDutyCycle(dutyCycle)
		
		# Update encoder states
		if (gpio.input(12) != buttonBR):
			buttonBR = int(gpio.input(12)) #holds the state
			counterBR += 1

		if (gpio.input(7) != buttonFL):
			buttonFL = int(gpio.input(7)) #holds the state
			counterFL += 1
	
	stopDriving()


def turnRight(turnAngle):
	global dutyCycle, leftPWMPin, rightPWMPin
	#Left wheels
	gpio.output(31, True)
	gpio.output(33, False)

	# Right Wheels
	gpio.output(35, True)
	gpio.output(37, False)	
	
	currentHeading = getIMUAngle()
	previousHeading = currentHeading

	desiredHeading = currentHeading + turnAngle

	if (desiredHeading < 360):
		while (currentHeading < desiredHeading):
			leftPWMPin.ChangeDutyCycle(dutyCycle)
			rightPWMPin.ChangeDutyCycle(dutyCycle)
			currentHeading = getIMUAngle()
	
	else:
		while (currentHeading < 360):
			leftPWMPin.ChangeDutyCycle(dutyCycle)
			rightPWMPin.ChangeDutyCycle(dutyCycle)
			previousHeading = currentHeading
			currentHeading = getIMUAngle()
			if (currentHeading < previousHeading):
				break
		while (currentHeading < desiredHeading - 360):
			leftPWMPin.ChangeDutyCycle(dutyCycle)
			rightPWMPin.ChangeDutyCycle(dutyCycle)
			currentHeading = getIMUAngle()
	
	stopDriving()


def turnLeft(turnAngle):
	global dutyCycle, counterBR, counterFL, leftPWMPin, rightPWMPin
	#Left wheels
	gpio.output(31, False)
	gpio.output(33, True)

	# Right Wheels
	gpio.output(35, False)
	gpio.output(37, True)	

	currentHeading = getIMUAngle()
	previousHeading = currentHeading
	if (currentHeading == None):
		return None

	desiredHeading = currentHeading - turnAngle

	if (desiredHeading > 0):
		while (currentHeading > desiredHeading):
			leftPWMPin.ChangeDutyCycle(dutyCycle)
			rightPWMPin.ChangeDutyCycle(dutyCycle)
			currentHeading = getIMUAngle()
	
	else:
		while (currentHeading > 0):
			leftPWMPin.ChangeDutyCycle(dutyCycle)
			rightPWMPin.ChangeDutyCycle(dutyCycle)
			previousHeading = currentHeading
			currentHeading = getIMUAngle()
			if (currentHeading > previousHeading):
				break
		while (currentHeading > desiredHeading + 360):
			leftPWMPin.ChangeDutyCycle(dutyCycle)
			rightPWMPin.ChangeDutyCycle(dutyCycle)
			currentHeading = getIMUAngle()
	
	stopDriving()


if __name__ == '__main__':
	##### Initialize GPIO pins ####
	gpio.setmode(gpio.BOARD)
	gpio.setup(31, gpio.OUT)  	# IN1
	gpio.setup(33, gpio.OUT) 	# IN2
	gpio.setup(35, gpio.OUT) 	# IN3
	gpio.setup(37, gpio.OUT) 	# IN4
	gpio.setup(38, gpio.OUT) 	# Left motor PWM pin
	gpio.setup(40, gpio.OUT) 	# Right motor PWM pin
	
	leftPWMPin = gpio.PWM(38,50)
	rightPWMPin = gpio.PWM(40,50) 

	leftPWMPin.start(0)
	rightPWMPin.start(0)

	gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP) 	# Back right encoder
	gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP) 		# Front left encoder

	dutyCycle = 15
	diff = 5

	################################# CLAW OPERATION ##############################
	# Claw parameters
	clawPin = 36
	#closePWM = 5.5
	closePWM = 7
	openPWM = 9

	# Setup RPi GPIO pins
	gpio.setup(clawPin, gpio.OUT)
	claw = gpio.PWM(clawPin, 50)		# Set PWM to 50 Hz

	# Start claw in open position
	claw.start(openPWM)

	############################### IMU SERIAL CONNECTION ############################
	# Create serial connection
	ser = serial.Serial('/dev/ttyUSB0', 9600)
	# Flush initial readings
	time.sleep(5)
	ser.reset_input_buffer()

	############################## VIDEO SETTINGS #############################

	# Set desired video resolution, framerate and logging offset
	resolution = (1280,720)
	fps = 15

	# Create video capture object 
	videoCapture = cv2.VideoCapture(0)
	videoCapture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
	videoCapture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
	videoCapture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
	videoCapture.set(cv2.CAP_PROP_FPS, 30)

	stopDriving()

	while (True):

		retrieveCommand = input('Retrieve next object (Enter Y or N)? ')

		if (retrieveCommand == 'Y'):
			ret, image = videoCapture.read()
			ret, image = videoCapture.read()
			ret, image = videoCapture.read()

			# Flip the frame both horizontally and vertically
			image = cv2.flip(image, -1)

			angle, distanceCM, newImage = getObjectLocation(image)

			cv2.imshow('Threshold', newImage)
			# Exit if the user presses 'q'
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
			
			print(angle)

			# Turn toward object
			while(abs(angle) < 2):
				if angle < 0:
					# Turn left if angle is negative
					turnLeft(-angle)
				else: 
					turnRight(angle)
			
			# Drive to object, pick it up and drive back
			driveForward((distanceCM - 10)/100)

			claw.ChangeDutyCycle(closePWM)

			driveBackward((distanceCM - 10)/100)

		else:
			break

		'''
		realDistance = ticks2dist((counterBR + counterFL)/2)
		y_val = realDistance*(math.sin(math.radians(currentAngle)))
		x_val = realDistance*(math.cos(math.radians(currentAngle))) 
		Y.append(y_val)
		X.append(x_val)
		f.write(str(x_val)+" "+str(y_val))
		'''
			
	leftPWMPin.stop()
	rightPWMPin.stop()
	claw.stop()
	stopDriving()
	gpio.cleanup()

	# Release video and file object handles
	videoCapture.release()

	print('Video object handle closed')