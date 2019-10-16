
from robot_servo import *
ttyUSB0 = serial_converter('/dev/ttyUSB0', 115200)
servo_list = [futaba_servo(ttyUSB0, id) for id in range(1,21)]
hand = servo_cluster()
hand.activate()


