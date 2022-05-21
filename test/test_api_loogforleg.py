from pycreate2 import Create2
import time


roomba = Create2('/dev/ttyUSB0', 115200)
roomba.start()
roomba.safe()

#######################################################
DRIVE_SPEED = 80
"""
    drive roomba forward while looking for leg
    stop driving when leg is detected
"""
def lookForLeg(): # Keep moving forward, and stop when leg is detected
    print("Looking for leg")
    roomba.drive_direct(DRIVE_SPEED, DRIVE_SPEED)
    
    continueDrive = True # Variable on whether we should keep moving forward

    while continueDrive:
        sensors = roomba.get_sensors()
        # time.sleep(0.5)
        print(sensors.bumps_wheeldrops[0], sensors.bumps_wheeldrops[1])
        if sensors.bumps_wheeldrops[0] or sensors.bumps_wheeldrops[1]: # If leg is detected, stop
            print("Table leg detected")
            roomba.drive_stop()
            continueDrive = False

#######################################################

lookForLeg()

roomba.drive_stop()
