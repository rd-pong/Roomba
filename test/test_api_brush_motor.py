from pycreate2 import Create2
import time


roomba = Create2('/dev/ttyUSB0', 115200)
roomba.start()
roomba.safe()

roomba.start_main_brush()  # start main brush motor only
time.sleep(5)
roomba.stop_main_brush()  # stop main brush motor
print("Finished")
