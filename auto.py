from droneapi.lib import VehicleMode
from pymavlink import mavutil
import time

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

print "sleep..."
time.sleep(10)
print "awaken..."

if v.mode.name == "AUTO":
	v.mode = VehicleMode("GUIDED")
	v.flush()
	print "Mode change to GUIDED"

print "sleep again..."
time.sleep(10)
print "awaken..."

v.mode = VehicleMode("AUTO")
v.flush()
print "Mode change to AUTO"
