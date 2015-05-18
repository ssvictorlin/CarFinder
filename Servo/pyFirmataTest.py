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
	time.sleep(0.5)
	val = val + 15
	if val == 90:
		val = 0
		dig6.write(switch)
		switch = switch + 1
		switch = switch % 2
	dig3.write(val)	


