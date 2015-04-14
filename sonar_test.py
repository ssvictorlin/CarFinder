from pyfirmata import Arduino, util
from droneapi.lib import VehicleMode
from pymavlink import mavutil
import time

# https://pypi.python.org/pypi/pyFirmata/1.0.2 for domeloading pyFirmata

# send_nav_velocity - send nav_velocity command to vehicle to request it fly in specified direction
# x direction: left/right, y direction: back/forth, z direction: up/down
def send_nav_velocity(vehicle, velocity_x, velocity_y, velocity_z):
    # create the SET_POSITION_TARGET_LOCAL_NED command
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0,       # type_mask (not used)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not used)
        0, 0)    # yaw, yaw_rate (not used)
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def send_nav_yaw(vehicle, yaw):
    # create the SET_POSITION_TARGET_LOCAL_NED command
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0,       # type_mask (not used)
        0, 0, 0, # x, y, z positions (not used)
        0, 0, 0, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not used)
        yaw, 0.2)    # yaw, yaw_rate (not used)
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

board = Arduino('/dev/ttyACM99')
time.sleep(1)

rightSonar = board.get_pin('a:1:i')
frontSonar = board.get_pin('a:2:i')
leftSonar  = board.get_pin('a:3:i')

it = util.Iterator(board)
it.start()
rightSonar.enable_reporting()
frontSonar.enable_reporting()
leftSonar.enable_reporting()

api = local_connect()
v = api.get_vehicles()[0]

print "Mode: %s" % v.mode
print "Armed: %s" % v.armed
print "groundspeed: %s" % v.groundspeed

cmds = v.commands
cmds.download()
cmds.wait_valid()
print "Home WP: %s" % cmds[0]
print "Current dest: %s" % cmds.next



while True:
	rightInchesAway = rightSonar.read()
	frontInchesAway = frontSonar.read()
	leftInchesAway = leftSonar.read()

	#print str(round(rightInchesAway*5/0.0098)*2.54) + ' cm from sonarRight'
	print str(round(frontInchesAway*5/0.0098)*2.54) + ' cm from sonarFront'
	#print str(round(leftInchesAway*5/0.0098)*2.54) + ' cm from sonarLeft'

	frontInchesAwayCM = (frontInchesAway*5/0.0098)*2.54

	modeG = "GUIDED"
	modeA = "ALT_HOLD"
	modeL = "LOITER"
	if frontInchesAwayCM <= 100:
		if v.mode.name == "LOITER":
			v.mode = VehicleMode("GUIDED")
			v.flush()
			print "Mode change to GUIDED"
			send_nav_yaw(v, 0.5)
		else:
			print "Still spinning..."
			send_nav_yaw(v, 0.5)
	else:
		if v.mode.name == "GUIDED":
			v.mode = VehicleMode("LOITER")
			v.flush()
			print "Mode change to LOITER"
		else:
			print "Mode stays in LOITER"
	time.sleep(2)

