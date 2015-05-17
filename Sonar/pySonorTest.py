from pyfirmata import Arduino, util
from droneapi.lib import VehicleMode
from pymavlink import mavutil
import time
import sys
import math
import string
# https://pypi.python.org/pypi/pyFirmata/1.0.2 for domeloading pyFirmata


max_speed = 0.5
min_speed = 0.05
speed_ratio = 20

guided_last_update = time.time()
vel_update_rate = 0.5
api = local_connect()
v = api.get_vehicles()[0]


# send_nav_velocity - send nav_velocity command to vehicle to request it fly in specified direction
# x direction: left/right, y direction: back/forth, z direction: up/down
def send_nav_velocity(vehicle, velocity_x, velocity_y, velocity_z):
    # create the SET_POSITION_TARGET_LOCAL_NED command
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111,       # type_mask (not used)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not used)
        0, 0)    # yaw, yaw_rate (not used) 
        # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

    
def condition_yaw(heading):
    # create the CONDITION_YAW command
    msg = vehicle.message_factory.mission_item_encode(0, 0,  # target system, target component
        0,     # sequence
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # frame
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,         # command
        2, # current - set to 2 to make it a guided command
	    0, # auto continue
	    heading, 0, 0, 0, 0, 0, 0) # param 1 ~ 7
        # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()
    
def get_speed(reading):
    speed = speed_ratio  * (1/reading)
    
    speed = max(speed, min_speed)
    speed = min(speed, max_speed)
    return speed

board = Arduino('/dev/ttyACM99')

rightSonar = board.get_pin('a:1:i')
frontSonar = board.get_pin('a:2:i')
leftSonar  = board.get_pin('a:3:i')

it = util.Iterator(board)
it.start()
rightSonar.enable_reporting()
frontSonar.enable_reporting()
leftSonar.enable_reporting()
time.sleep(1)

print "Mode: %s" % v.mode
print "Armed: %s" % v.armed
print "groundspeed: %s" % v.groundspeed

count = 0

while True:
	#rightInchesAway = rightSonar.read()

	frontInchesAway = frontSonar.read()
	#leftInchesAway = leftSonar.read()

	#print str(round(rightInchesAway*5/0.0098)*2.54) + ' cm from sonarRight'
	print str(round(frontInchesAway*5/0.0098)*2.54) + ' cm from sonarFront'
	#print str(round(leftInchesAway*5/0.0098)*2.54) + ' cm from sonarLeft'

	frontInchesAwayCM = (frontInchesAway*5/0.0098)*2.54
	
	# get current time
        now = time.time()

        # exit immediately if it's been too soon since the last update
        if (now - guided_last_update) < vel_update_rate:
            print "too soon, I can't respond..."
	    
	if frontInchesAwayCM <= 100:

		if count < 3:
			count= count+1
		else:
		    s = get_speed(frontInchesAwayCM)
		    if v.mode.name != "GUIDED":
			    v.mode = VehicleMode("GUIDED")
			    v.flush()
			    print "Mode change to GUIDED and moving in speed : %f" % s
			    send_nav_velocity(v,0, s, 0)
			    
		    else:
			    print "moving... in speed : %f" % s
			    send_nav_velocity(v,0, s, 0)
			    
	else:
		count = 0
		print "no danger..."
		if v.mode.name == "GUIDED":
			send_nav_velocity(v,0, 0, 0)
		
		else:
			print "Mode in LOITER"
	guided_last_update = time.time()
	time.sleep(0.5)
sys.exit()
