from pycreate2 import Create2
import time

roomba = Create2('/dev/ttyUSB0', 115200)
roomba.start()
roomba.safe()

for i in range(500):
    sensors = roomba.get_sensors()
    print(i, sensors.bumps_wheeldrops[0], sensors.bumps_wheeldrops[1])
    time.sleep(0.1)
