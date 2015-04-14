import sys
import time
import cv2
import numpy
import math
from pymavlink import mavutil

class MoveToCar:

	# FIXME - check if vehicle altitude is too low
    # FIXME - check if we are too far from the desired flightplan
	
	# find the yaw angle from the car position in image
	def find_car_direction(self, xpos, ypos, )



	# calculate velocity vector (Vx, Vy, Vz) from the pitch and yaw angle and the speed as 
	#     the magnitude 
	def get_vel_vector(self, pitch, yaw, speed):
		cos_pitch = math.cos(pitch)
		x = speed * math.cos(yaw) * cos_pitch
		y = speed * math.sin(yaw) * cos_pitch
		z = speed * math.sin(pitch)
		return x, y, z

	# send_nav_velocity - send nav_velocity command to vehicle to request it fly in specified direction
    def send_nav_velocity(self, velocity_x, velocity_y, velocity_z):
        # create the SET_POSITION_TARGET_LOCAL_NED command
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                                                     0,       # time_boot_ms (not used)
                                                     0, 0,    # target system, target component
                                                     mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
                                                     0,       # type_mask (not used)
                                                     0, 0, 0, # x, y, z positions (not used)
                                                     velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
                                                     0, 0, 0, # x, y, z acceleration (not used)
                                                     0, 0)    # yaw, yaw_rate (not used) 
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
