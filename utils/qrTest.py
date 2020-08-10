import pyzbar.pyzbar as pyzbar
import numpy as np
import cv2


def decode(im) : 
    # Find barcodes and QR codes
    decodedObjects = pyzbar.decode(im)

    # Print results
    for obj in decodedObjects:
        print('Type : ', obj.type)
        print('Data : ', obj.data,'\n')
        
    return decodedObjects


# Display barcode and QR code location  
def display(im, decodedObjects):

    # Loop over all decoded objects
    for decodedObject in decodedObjects: 
        points = decodedObject.polygon

        # If the points do not form a quad, find convex hull
        if len(points) > 4 : 
            hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
            hull = list(map(tuple, np.squeeze(hull)))
        else : 
            hull = points
        
        # Number of points in the convex hull
        n = len(hull)

        # Draw the convext hull
        for j in range(0,n):
            cv2.line(im, hull[j], hull[ (j+1) % n], (255,0,0), 3)

    # Display results 
    cv2.imshow("Results", im)
    cv2.waitKey(0)

  
# Main 
if __name__ == '__main__':
	## Raspberry Pi Camera v2 Specifications
	# Resolution: 	3280 × 2464
	# Sensor Size: 	3.68 x 2.76 mm
	# Focal Length: 3.04 mm

	sensorSizeY = 2.76	    # mm
	sensorResY = 2464	    # px
	focalLength = 3.04	    # mm
	objectHeight = 3.175	# cm
	# Account for 2x2 binning if resolution is (1280, 720)
	# https://picamera.readthedocs.io/en/release-1.12/fov.html
	pixPerMil = (sensorResY/2) / sensorSizeY	# px/mm


	imagePath = '../Test_Images/QRTest1.jpg'


	# Set desired video resolution, framerate and logging offset
	resolution = (1280,720)
	fps = 15

	# Create video capture object 
	videoCapture = cv2.VideoCapture(0)
	videoCapture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
	videoCapture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
	videoCapture.set(cv2.CAP_PROP_FPS, 30)


	while(videoCapture.isOpened()):

		# Grab the current frame
		ret, image = videoCapture.read()
		
		# Flip images about both axes (rotate 90 deg)
		image = cv2.flip(image, -1)

		print(image.shape)

		# Read image
		#image = cv2.imread(imagePath)

		decodedObjects = pyzbar.decode(image)

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

		# Display results 
		cv2.imshow("Results", image)

		# Exit if the user presses 'q'
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	# Release video and file object handles
	videoCapture.release()

	print('Video object handle closed')


