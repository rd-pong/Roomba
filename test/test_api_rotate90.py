from pycreate2 import Create2
import time


roomba = Create2('/dev/ttyUSB0', 115200)
roomba.start()
roomba.safe()

####################################################################################
DRIVE_SPEED = 50

"""
    drive roomba rotate 90 degrees
    in clockwise direction or counterclockwise direction
"""
def rotate90(clockwise): # rotate 90 deg
    sensors = roomba.get_sensors()
    time.sleep(0.5)

    angleRotated = 0 # variable to calculated amount of rotation
    
    if clockwise == 0: # ccw
        print("Detected, rotate ccw")
        roomba.drive_direct(DRIVE_SPEED, -DRIVE_SPEED)
        while angleRotated < 90:
            sensors = roomba.get_sensors()
            print(sensors.angle)
            time.sleep(.1)
            angleRotated += abs(sensors.angle)
            print("Angle Rotated: ", angleRotated)
    else: # cw
        print("Detected, rotate cw")
        roomba.drive_direct(-DRIVE_SPEED, DRIVE_SPEED)
        while angleRotated < 90:
            sensors = roomba.get_sensors()
            time.sleep(.1)
            angleRotated += abs(sensors.angle)
            # print("Angle Rotated: ", angleRotated)
    roomba.drive_stop()

def rotate90time():
    roomba.drive_direct(DRIVE_SPEED, -DRIVE_SPEED)
    time.sleep(3.3)
    roomba.drive_stop()

####################################################################################
rotate90(0) # ccw
roomba.drive_stop()
