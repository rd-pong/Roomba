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

roomba.start_main_brush()
time.sleep(5)
roomba.stop_main_brush()
print("Finished")

#######################################################

# roomba.drive_stop()
