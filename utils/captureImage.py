from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

resolution = (640, 480)
#resolution = (1280, 720)
#resolution = (1920, 1080)

# Initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = resolution
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=resolution)

# Allow the camera to warmup
time.sleep(0.1)

# Capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# Grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array

	# Flip images about both axes (rotate 90 deg)
	image = cv2.flip(image, -1)

	# Show the frame
	cv2.imshow("Frame", image)
	key = cv2.waitKey(1) & 0xFF

	# Clear the stream in preparation for the next frame
	rawCapture.truncate(0)
	#rawCapture.seek(0)

	# If the s key is pressed, save the image and break from the loop
	if key == ord("s"):
		cv2.imwrite('image.jpg', image)
		break
	
	# If the q key is pressed, exit the loop
	elif key == ord("q"):
		break

# Turn off camera object
camera.close()
