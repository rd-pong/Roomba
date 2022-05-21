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

for i in range(40):
    sensors = roomba.get_sensors()
    print(i, sensors.bumps_wheeldrops[0], sensors.bumps_wheeldrops[1])
    time.sleep(0.5)

#######################################################

roomba.drive_stop()
