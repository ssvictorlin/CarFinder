import sys
import time
import datetime
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

        # state variable: 0 - searching target (AUTO), 1 - target found but waiting to double check (LOITER), 
        # 2 - target locked (GUIDED), 3 - taking photos of target (CIRCLE) 
        self.vehicle_state = 0
        # for the start, initial state = 1 when we set up the position and switch to AUTO mode, it starts
        # the main script 
        self.initial_state = 1

        # some thresholds to be tuned
        self.max_target_area = 6000
        self.min_target_area = 2500
        self.max_speed = 1.5
        self.min_speed = 0.2
        self.speed_ratio = 1/250
    
        # target info
        self.target_found = False
        self.xpos = None
        self.ypos = None
        self.dist = 500
        self.guided_target_vel = None
        self.speed = 0.001
        self.d_threshold = 40

        # missing counts
        self.lost_count = 0
    
        # finding counts
        self.find_count = 0

        # open camera
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,self.img_width)
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,self.img_height)
    
        # print to file
        self.filename = "Log: " + datetime.datetime.now().strftime("%m-%d %H:%M:%S") + ".txt"
        self.f = open("Logs/" + self.filename, 'w')

        # check if we can connect to camera
        if not self.camera.isOpened():
            print "failed to open camera, exiting!"
            sys.exit(0)
        print "camera is open..."
        print >> self.f, "[" + datetime.datetime.now().strftime("%m-%d %H:%M:%S:%f") + "]" + "camera is open..."

    # find target position in image
    def search_target(self):    
        # get the image from the webcam
        success_flag, image = self.camera.read()

        # if successful overwrite our latest image
        if success_flag:
            latest_image = image

            # search target
            self.red_dectection(image)
        
            # target found and we stop immediately to double check
            if self.target_found:
                print "target found..."
                print >> self.f, "[" + datetime.datetime.now().strftime("%m-%d %H:%M:%S:%f") + "]" + "target found..."
        
                # mode change to LOITER
                self.v.mode = VehicleMode("LOITER")
                self.v.flush()
                self.guided_target_vel = self.get_vel_vector(self.xpos, self.ypos, self.speed)
                print >> self.f, "[" + datetime.datetime.now().strftime("%m-%d %H:%M:%S:%f") + "]" + "-> LOITER..."
                # change the state to 1 double check
                self.vehicle_state = 1
            else:
                print "still searching target..."
                print >> self.f, "[" + datetime.datetime.now().strftime("%m-%d %H:%M:%S:%f") + "]" + "still searching target..."        
        else:
            print "can't access to the camera..." 
            print >> self.f, "[" + datetime.datetime.now().strftime("%m-%d %H:%M:%S:%f") + "]" + "can't access to the camera..."


    # detecting function
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
            print "the area is %f ..." % areas[max_index]
            Area = areas[max_index]
        
            #### FIXME: need a check on whether it is a target
            # using the the area as a threshold
            # target is found
            if areas[max_index] > min_target_area & areas[max_index] < max_target_area :
                # mark it down
                self.target_found = True
                print "target is found..."
        
                # write the image to file to check the results: 1. Searching 2. Moving 
                cv2.rectangle(frame ,(x,y), (x+w,y+h), (0, 255,0), 2)
                if self.vehicle_state == 0:
                    filename = "Searching: " + datetime.datetime.now().strftime("%m-%d %H:%M:%S--") + "x:"+ str(self.xpos) +" y:"+ str(self.ypos) + " A:" + str(Area) + ".jpg"
                    cv2.imwrite(filename, frame)
                elif self.vehicle_state == 2:
                    filename = "Moving: " + datetime.datetime.now().strftime("%m-%d %H:%M:%S--") + "x:"+ str(self.xpos) +" y:"+ str(self.ypos) + " A:" + str(Area) + ".jpg"
                    cv2.imwrite(filename, frame)
            else:
                # target is too small or large to be a valid target
                print "non-valid area..."
                print >> self.f, "[" + datetime.datetime.now().strftime("%m-%d %H:%M:%S:%f") + "]" + "non-valid area..."
                self.target_found = False
                self.xpos = None
                self.ypos = None
        else:
            # target is not found
            self.target_found = False
            print "no target yet..."
            self.xpos = None
            self.ypos = None

    # calculate the proper speed
    def get_speed(distance):
        speed = speed_ratio  * distance
    
        speed = max(speed, min_speed)
        speed = min(speed, max_speed)
        return speed

    # calculate velocity vector (Vx, Vy, Vz) from target position and the speed as the magnitude 
    def get_vel_vector(self, xpos, ypos, speed):
        x = speed * (xpos - self.img_center_x)/self.dist
        y = speed * (ypos - self.img_center_y)/self.dist
        z = 0
        return x, y, z
    
    # check distance everytime we move to the target
    def check_distance(self, xpos, ypos):
        d_now = math.sqrt(math.pow(xpos - self.img_center_x, 2) + math.pow(ypos - self.img_center_y, 2))
        print "distance is %f ..." % d_now
        if d_now < self.d_threshold:
            return (d_now, True)
        else:
            return (d_now, False)

    # send_nav_velocity - send nav_velocity command to vehicle to request it fly in specified direction
    def send_nav_velocity(self, velocity_x, velocity_y, velocity_z):
        # create the SET_POSITION_TARGET_LOCAL_NED command
        msg = self.v.message_factory.set_position_target_local_ned_encode(
                                                     0,       # time_boot_ms (not used)
                                                     0, 0,    # target system, target component
                                                     mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
                                                     0b0000111111000111,       # type_mask (not used)
                                                     0, 0, 0, # x, y, z positions (not used)
                                                     velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
                                                     0, 0, 0, # x, y, z acceleration (not used)
                                                     0, 0)    # yaw, yaw_rate (not used) 
        # send command to vehicle
        self.v.send_mavlink(msg)
        self.v.flush()
    
    # move to target by the calculated velocity vector
    #### FIXME: the target might be gone sometimes when we are moving, so how do we fix this?
    def move_to_target(self):
        success_flag, image = self.camera.read()

        # if successful overwrite our latest image
        if success_flag:
            latest_image = image

            # search target
            self.red_dectection(image)

            # get current distance
            self.dist, close_enough = self.check_distance(self.xpos, self.ypos)
        
            if self.target_found:
                #print "target locked..." 
                self.speed = self.get_speed(self.dist)
        
                # check distance of the target to us
                if close_enough:
                    print "coming really close..."
                    print >> self.f, "[" + datetime.datetime.now().strftime("%m-%d %H:%M:%S:%f") + "]" + "coming really close..."
            
                    self.v.mode = VehicleMode("LOITER")
                    self.v.flush()
                    print >> self.f, "[" + datetime.datetime.now().strftime("%m-%d %H:%M:%S:%f") + "]" + "-> LOITER..."
            
                    # change the state to complete!
                    self.vehicle_state = 3
            
                    filename = "Final: " + datetime.datetime.now().strftime("%m-%d %H:%M:%S--") + "x:" + str(self.xpos) +" y:"+ str(self.ypos) + " A:" + str(Area) + ".jpg"
                    cv2.imwrite(filename, latest_image)
                else:
                    # calculate the velocity vector
                    self.guided_target_vel = self.get_vel_vector(self.xpos, self.ypos, self.speed)
                    now = time.time()
                    self.send_nav_velocity(self.guided_target_vel[0], self.guided_target_vel[1], self.guided_target_vel[2])
                    next = time.time()
            
                    print "moving... in speed %f ... for %f secs " % (self.speed, next - now)
                    print >> self.f, "[" + datetime.datetime.now().strftime("%m-%d %H:%M:%S:%f") + "]" + "moving... in speed %f ... for %f secs " % (self.speed, next - now)
                
            else:
                # check three times before giving up this target
                if self.lost_count < 3:
                    self.lost_count = self.lost_count + 1
                    print >> self.f, "[" + datetime.datetime.now().strftime("%m-%d %H:%M:%S:%f") + "]" + "not found count %d " % self.lost_count
                # can't find the target, back to searching
                else:
                    print "lost target..."
                    print >> self.f, "[" + datetime.datetime.now().strftime("%m-%d %H:%M:%S:%f") + "]" + "lost target..."
            
                    # save the image to see why for real cars
                    filename = "Lost: " + datetime.datetime.now().strftime("%m-%d %H:%M:%S") + ".jpg"
                    cv2.imwrite(filename, latest_image)
            
                    # shoud be back to searching state, but for experiment, we need to stop it for safety  
                    self.vehicle_state = 4
                    self.lost_count = 0
                    print "stop mission because we lost target ..."
                    print >> self.f, "[" + datetime.datetime.now().strftime("%m-%d %H:%M:%S:%f") + "]" + "stop mission because we lost target ..."
        else: 
            print "can't access to the camera..."

    # mission complete
    def complete(self):
        if self.v.mode.name != "LOITER":
            self.v.mode = VehicleMode("LOITER")
            self.v.flush()
            print >> self.f, "[" + datetime.datetime.now().strftime("%m-%d %H:%M:%S:%f") + "]" + "-> LOITER..."
        
        print "I think I am on top of it!"
        print >> self.f, "[" + datetime.datetime.now().strftime("%m-%d %H:%M:%S:%f") + "]" + "I think I am on top of it!"
    
    # main
    def run(self):
        print "initializing..."
        while not self.api.exit:
            # still setting up the position
            if self.initial_state == 1:
                if self.v.mode.name == "LOITER":
                    self.initial_state = 0
                    print "running main script..."
                    print >> self.f, "[" + datetime.datetime.now().strftime("%m-%d %H:%M:%S:%f") + "]" + "running main script..."
            # start the mission        
            else:
                # searching for target
                if self.vehicle_state == 0:
                    self.search_target()

                # find target 
                elif self.vehicle_state == 1:
                    self.v.mode = VehicleMode("GUIDED")
                    self.v.flush()
                    print "Mode change to GUIDED..."
                    print >> self.f, "[" + datetime.datetime.now().strftime("%m-%d %H:%M:%S:%f") + "]" + "-> GUIDED..."
                    self.vehicle_state = 2

                # moving toward target
                elif self.vehicle_state == 2:
                    self.move_to_target()

                # mission complete
                elif self.vehicle_state == 3:
                    self.complete()
            
                else:
                    sys.exit(0)

            time.sleep(0.5)
            
Move = MoveToCar()
Move.run()


        

           
          
