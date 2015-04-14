import cv2
import numpy as np
import sys

while(1):
	# Get user supplied values
	imagePath = sys.argv[1]

	# Read the image
	frame = cv2.imread(imagePath)

	# Convert BGR to HSV
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	# define range of red color in HSV
	lower_red = np.array([0,100,100])
	upper_red = np.array([10,255,255])

	# Threshold the HSV image to get only red colors
	mask = cv2.inRange(hsv, lower_red, upper_red)

	# Bitwise-AND mask and original image
	res = cv2.bitwise_and(frame,frame, mask= mask)

	#cv2.imshow('frame',frame)
	#cv2.imshow('mask',mask)
	#cv2.imshow('res',res)
    
	res = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
	#cv2.imshow('1', res) 
	contours, hier = cv2.findContours(res, 2, 2)

	areas = [ cv2.contourArea(c) for c in contours]
	max_index = np.argmax(areas)
	cnt = contours[max_index]
	x, y, w, h = cv2.boundingRect(cnt)
	#print "x = %f" % x
	#print "y = %f" % y
	#print "w = %f" % w
	#print "h = %f" % h
	
	cv2.rectangle(frame ,(x,y), (x+w,y+h), (0, 255,0), 2)
	cv2.imshow('result', frame)
	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break

cv2.destroyAllWindows()

