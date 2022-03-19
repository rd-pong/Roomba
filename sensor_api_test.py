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

# roomba.drive_direct(200, 200)
# hit_obstacle = False
# while not hit_obstacle:
#     sensors = roomba.get_sensors()
#     # if sensors.light_bumper[0] or sensors.light_bumper[1] or sensors.light_bumper[2] or sensors.light_bumper[3] or sensors.light_bumper[4] or sensors.light_bumper[5]:
#     # if sensors.wall:
#     # if sensors.cliff_front_left or sensors.cliff_front_right or sensors.cliff_left or sensors.cliff_right:
#     if sensors.bumps_wheeldrops[0] or sensors.bumps_wheeldrops[1]:
#         hit_obstacle = True

for i in range(40):
    sensors = roomba.get_sensors()
    print(i, sensors.bumps_wheeldrops[0], sensors.bumps_wheeldrops[1])
    time.sleep(0.5)

# roomba.drive_direct(150, 150)

# continueDrive = True # Variable on whether we should keep moving forward

# while continueDrive:
#     sensors = roomba.get_sensors()
#     if sensors.bumps_wheeldrops[0] or sensors.bumps_wheeldrops[1]: # If leg is detected, stop
#         roomba.drive_stop()
#         continueDrive = False


# startTime = time.time()
# angleRotated = 0

# while angleRotated < 95:
#     sensors = roomba.get_sensors()
#     roomba.drive_direct(-50, 50)
#     print("+", abs(sensors.angle))
#     angleRotated += abs(sensors.angle)
#     print(angleRotated)
#     time.sleep(0.1)

#######################################################

roomba.drive_stop()
