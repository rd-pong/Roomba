from pycreate2 import Create2
import time

def showBattery():
    sensor = roomba.get_sensors()
    print(sensor.battery_charge)
    print(sensor.battery_capacity)
    print("battery: ", sensor.battery_capacity*100/3000, "%")

roomba = Create2('/dev/ttyUSB0', 115200)
roomba.start()
roomba.safe()


# showBattery()
#######################################################



#######################################################

roomba.drive_stop()
