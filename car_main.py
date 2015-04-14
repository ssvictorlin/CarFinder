import time 
import math
from pymavlink import mavutil
from droneapi.lib import VehicleMode, Location
from car_detect import car_detector
from car_video import my_webcam


class CarMain(object):
	def __init__(self):

		## DroneAPI
        # First get an instance of the API endpoint (the connect via web case will be similar)
        self.api = local_connect()

        # Our vehicle (we assume the user is trying to control the virst vehicle attached to the GCS)
        self.vehicle = self.api.get_vehicles()[0]

    def check_video_out(self):
    	self.my_webcam.



    def car_recognition(self):

    	# record time
    	now = time.time()

    	
	def run(self):
		# open camera when we are running this mission

		while not self.api.exit:

			# only start mission when the home is set
			if self.check_home():

				if self.vehicle.mode.name == "AUTO":
					# start image capturing 
					self.check_video_out()

					# recognize cars in image
					self.car_recognition()

				# if odroid u3 is in control, guide it to the car
				elif self.vehicle.mode.name == "GUIDED":
					self.move_to_car()

				# check if it is illegally parked 
				elif self.vehicle.mode.name == "LOITER":
					self.parking_line_detect()

				# Capture 360-degree photos
				elif self.vehicle.mode.name == "CIRCLE":
					self.circle_photo_taker()

			time.sleep(0.05)




start = CarMain()
start.run()
				
