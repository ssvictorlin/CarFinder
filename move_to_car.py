import sys
import time
import cv2
import numpy as np
import math
from pymavlink import mavutil
from droneapi.lib import VehicleMode


class MoveToCar:

	# FIXME - check if vehicle altitude is too low
	# FIXME - check if we are too far from the desired flightplan
    def __init__(self):

        # set up the connection with pixhawk
        self.api = local_connect()
        self.v = self.api.get_vehicles()[0]

        # set image resolution
        self.img_width = 640
        self.img_height = 480

        # get image center
        self.img_center_x = self.img_width / 2
        self.img_center_y = self.img_height / 2
        
        # define field of view
        self.cam_hfov = 70.42
        self.cam_vfov = 43.3

        # target info
        self.target_found = False
        self.xpos = None
        self.ypos = None
        self.guided_target_vel = None
        self.speed = 0.001
        # open camera
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,self.img_width)
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,self.img_height)

        # check if we can connect to camera
        if not self.camera.isOpened():
            print "failed to open camera, exiting!"
            sys.exit(0)
        print "camera is open..."

	# find target position in image
    def search_target(self):    
        # get the image from the webcam
        success_flag, image = self.camera.read()

        # if successful overwrite our latest image
        if success_flag:
            latest_image = image

            # search target
            self.red_dectection(image)

            if self.target_found:
                print "target found..."
                self.v.mode = VehicleMode("LOITER")
                self.v.flush()
                self.guided_target_vel = self.get_vel_vector(self.xpos, self.ypos, self.speed)
            else:
                print "still searching target..."        
        else:
            print "can't access to the camera..." 


    def red_dectection(self, frame):
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # define range of red color in HSV
        lower_red = np.array([0,100,100])
        upper_red = np.array([10,255,255])

        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame,frame, mask= mask)
        
        res = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        contours, hier = cv2.findContours(res, 2, 2)

        areas = [ cv2.contourArea(c) for c in contours]
        if areas:
            max_index = np.argmax(areas)
            cnt = contours[max_index]
            x, y, w, h = cv2.boundingRect(cnt)
            self.xpos = x + w/2
            self.ypos = y + h/2
            #print "x = %f" % x
            #print "y = %f" % y
            #print "w = %f" % w
            #print "h = %f" % h

            #### FIXME: need a check on whether it is a target
            #### maybe using the the area as a threshold
            # target is found
            self.target_found = True            
        else:
            # target is not found
            self.target_found = False
            self.xpos = None
            self.ypos = None     
        #cv2.rectangle(frame ,(x,y), (x+w,y+h), (0, 255,0), 2)
        #cv2.imshow('result', frame)
        #k = cv2.waitKey(5) & 0xFF
        #if k == 27:
        #    break
        
	# calculate velocity vector (Vx, Vy, Vz) from target position and the speed as the magnitude 
    def get_vel_vector(self, xpos, ypos, speed):
		x = speed * (xpos - self.img_center_x)
		y = speed * (ypos - self.img_center_y)
		z = 0
		return x, y, z

	# send_nav_velocity - send nav_velocity command to vehicle to request it fly in specified direction
    def send_nav_velocity(self, velocity_x, velocity_y, velocity_z):
        # create the SET_POSITION_TARGET_LOCAL_NED command
        msg = self.v.message_factory.set_position_target_local_ned_encode(
                                                     0,       # time_boot_ms (not used)
                                                     0, 0,    # target system, target component
                                                     mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
                                                     0,       # type_mask (not used)
                                                     0, 0, 0, # x, y, z positions (not used)
                                                     velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
                                                     0, 0, 0, # x, y, z acceleration (not used)
                                                     0, 0)    # yaw, yaw_rate (not used) 
        # send command to vehicle
        self.v.send_mavlink(msg)
        self.v.flush()

    def move_to_target(self):
        success_flag, image = self.camera.read()

        # if successful overwrite our latest image
        if success_flag:
            latest_image = image

            # search target
            self.red_dectection(image)
            if self.target_found:
                print "target locked..."
                self.guided_target_vel = self.get_vel_vector(self.xpos, self.ypos, self.speed)
                self.send_nav_velocity(self.guided_target_vel[0], self.guided_target_vel[1], self.guided_target_vel[2])
				print "moving..."
            else:
                # can't find the target, back to searching
                print "lost target..."
                self.v.mode = VehicleMode("AUTO")
                self.v.flush()
                print "Mode change back to AUTO..."
        else: 
            print "can't access to the camera..."
	def complete(slef):
        print "I think I am on top of it!"

    def run(self):
        print "running main script..."
        while not self.api.exit:
            if self.v.mode.name == "AUTO":
                self.search_target()

            elif self.v.mode.name == "LOITER":
                self.v.mode = VehicleMode("GUIDED")
                self.v.flush()
                print "Mode change to GUIDED..."

            elif self.v.mode.name == "GUIDED":
                self.move_to_target()

            time.sleep(0.5)
            
Move = MoveToCar()
Move.run()


        

           
          
