import cv2
import cv
import sys
import numpy as np
import math

# Get user supplied values
imagePath = sys.argv[1]
cascPath = "cascade.xml"



class CarDetector(object):
	def calcLength(self, x1, y1, x2, y2):
		return math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))

	def main(self, frame):

		car_found = False
		car_pos_x = 0
		car_pos_y = 0
		car_pos_w = 0
		car_pos_h = 0

		# Create the haar cascade
		faceCascade = cv2.CascadeClassifier(cascPath)

		# Read the image
		image = cv2.imread(imagePath)
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		# Rescale the input image
		scale = 6
		(imageH, imageW) = image.shape[:2]
		i1 = imageW/scale
		i2 = imageH/scale
		new = cv2.resize( gray, (i1, i2) ,fx=0, fy=0, interpolation = cv2.INTER_LINEAR )
		new = cv2.equalizeHist(new)

		# Detect faces in the image
		faces = faceCascade.detectMultiScale(
			new,
			scaleFactor=1.1,
			minNeighbors=2,
			minSize=(30, 30),
			flags = cv2.cv.CV_HAAR_SCALE_IMAGE
		)

		print "Found {0} faces!".format(len(faces))

		# Draw a rectangle around the faces
		for (x, y, w, h) in faces:
			
			x = x*scale
			y = y*scale
			w = w*scale
			h = h*scale
			#print "x= {0} , y = {1}".format(x, y)
			cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)

		# Min distance of car and the center in image 
		Lmin = self.calcLength(imageW, imageH, 0, 0)
		# Find the car that is closest to the center(to the quadcopter)
		if  len(faces) > 1:
			car_found = True
			for (x, y, w, h) in faces:
				x = x*scale
				y = y*scale
				w = w*scale
				h = h*scale
				L = self.calcLength(imageW/2, imageH/2, x+w/2, y+h/2)
				if L < Lmin: 
					Lmin = L
					car_pos_x = x
					car_pos_y = y
					car_pos_w = w
					car_pos_h = h
		# only one car in image
		elif len(faces) == 1:
			car_found = True
			car_pos_x = x
			car_pos_y = y
			car_pos_w = w
			car_pos_h = h
		# no car is found
		else:
			car_found = False
			car_pos_x = 0
			car_pos_y = 0
			car_pos_w = 0
			car_pos_h = 0

		cv2.imshow("Faces found", image)
		cv2.waitKey(0)
		return car_found, car_pos_x, car_pos_y, car_pos_h, car_pos_w


	

# create a single global object
car_detector = CarDetector()

# run a test if this file is being invoked directly from the command line
if __name__ == "__main__":
    car_detector.main()
