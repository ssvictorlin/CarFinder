from pyfirmata import Arduino, util
import time

board = Arduino('/dev/ttyACM99')
time.sleep(1)

dig6 = board.get_pin('d:6:o')
dig3 = board.get_pin('d:3:s')
alg5 = board.get_pin('a:5:i')

it = util.Iterator(board)
it.start()
alg5.enable_reporting()

val = 0
switch = 0

while True:
	time.sleep(0.01)
	val = val + 1
	if val == 180:
		val = 0
		dig6.write(switch)
		switch = switch + 1
		switch = switch % 2
		sonar1 = alg5.read()
		print str(round((sonar1*5/0.0098)*2.54, 3)) + ' cm from sonar1'
	dig3.write(val)	


