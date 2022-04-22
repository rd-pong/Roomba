from pycreate2 import Create2
import time

def showBattery(sensors):
    print(sensor.battery_charge)
    print(sensor.battery_capacity)
    print("battery: {}%".format(str(sensor.battery_charge*100/sensor.battery_capacity)[0:4]))

roomba = Create2('/dev/ttyUSB0', 115200)
roomba.start()
roomba.safe()

sensor = roomba.get_sensors()
showBattery(sensor)
#######################################################



#######################################################

roomba.drive_stop()
