import qwiic_vl53l1x
import qwiic_tca9548a
import serial
import time

from pycreate2 import Create2

DRIVE_SPEED = 100
ROTATE_SPEED = 50

"""
    drive roomba rotate 90 degrees
    in clockwise direction or counterclockwise direction
"""
def rotate90(clockwise): # rotate 90 deg
    sensors = roomba.get_sensors()
    angleRotated = 0 # variable to calculated amount of rotation
    
    if clockwise == 0: # ccw
        print("Detected, rotate ccw")
        roomba.drive_direct(DRIVE_SPEED, -DRIVE_SPEED)
        while angleRotated < 90:
            sensors = roomba.get_sensors()
            angleRotated += abs(sensors.angle)
            print("Angle Rotated: ", angleRotated)
    else: # cw
        print("Detected, rotate cw")
        roomba.drive_direct(-DRIVE_SPEED, DRIVE_SPEED)
        while angleRotated < 90:
            sensors = roomba.get_sensors()
            angleRotated += abs(sensors.angle)
            print("Angle Rotated: ", angleRotated)
    roomba.drive_stop()

"""
# Another way to rotate 90 degrees
def rotate90(clockwise):
    if clockwise == 0: # ccw
        print("Rotate ccw")
        roomba.drive_direct(ROTATE_SPEED, -ROTATE_SPEED)
        time.sleep(3.1)
    else: # cw
        print("Rotate cw")
        roomba.drive_direct(-ROTATE_SPEED, ROTATE_SPEED)
        time.sleep(3.1)
    roomba.drive_stop()
"""

"""
    drive roomba forward for a short distance
    will quit disinfection mode when hit obstacle
"""
def forwardShort():
    global stopDisinfection
    startTime = time.time()
    roomba.drive_direct(DRIVE_SPEED, DRIVE_SPEED)
    print("Move forward a short distance")

    continueDrive = True

    while continueDrive:
        sensors = roomba.get_sensors()
        # time.sleep(0.5)
        # print(time.time(), "\t", sensors.bumps_wheeldrops[0], sensors.bumps_wheeldrops[1])
        if sensors.bumps_wheeldrops[0] or sensors.bumps_wheeldrops[1]:
            print("Obstacle detected, stop disinfection")
            roomba.drive_stop()
            continueDrive = False
            stopDisinfection = True
            break
        if time.time() - startTime > 4:
            roomba.drive_stop()
            continueDrive = False


"""
    drive roomba backword for a short distance
"""
def backwardShort():
    print("Move backward a short distance")
    startTime = time.time()
    roomba.drive_direct(-DRIVE_SPEED, -DRIVE_SPEED)

    continueDrive = True
    while continueDrive:
        if time.time() - startTime > 2:
            roomba.drive_stop()
            continueDrive = 0


"""
    drive roomba forward while under the table
    until outside table based on ToF measured distance
"""
def forwardUntilOutsideTable(): # move forward until outside table based on ToF
    global stopDisinfection
    print("Move forward until outside table based on ToF")
    roomba.drive_direct(DRIVE_SPEED, DRIVE_SPEED)

    continueDrive = True # Variable on whether we should keep moving forward
    underTable = True # Variable on whether robot is under a table or not

    for _ in range(2): # throw first 20 measurements
        getFrontTof()

    while continueDrive:
        sensors = roomba.get_sensors()
        # time.sleep(0.5)
        # print(time.time(), "\t", sensors.bumps_wheeldrops[0], sensors.bumps_wheeldrops[1])
        if sensors.bumps_wheeldrops[0] or sensors.bumps_wheeldrops[1]: # If detect a table leg, we should stop disinfecting
            print("Obstacle detected, stop disinfection")
            roomba.drive_stop()
            continueDrive = False
            stopDisinfection = True
            break

        if underTable == True: # Table detected
            if getFrontTof() == 0: # If table no longer detected
                underTable = False
                startTime = time.time() # Store time since table first not detected
        else: # No table detected
            if time.time() - startTime > 2: # Has been out of table for 2sec
                roomba.drive_stop()
                continueDrive = False


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
        # print(time.time(), "\t", sensors.bumps_wheeldrops[0], sensors.bumps_wheeldrops[1])
        if sensors.bumps_wheeldrops[0] or sensors.bumps_wheeldrops[1]: # If leg is detected, stop
            print("Table leg detected")
            roomba.drive_stop()
            continueDrive = False
            break


"""
    align to table using ToF sensors
"""
def alignToTable(): # Align robot to table when facing to it using ToF sensor
    print("Align to table ...")
    global stopDisinfection
    
    for _ in range(20):
        getRightTof()
        getLeftTof()
   
    continueDrive = 1 # Variable on whether we should keep moving forward
    while continueDrive:
        sensors = roomba.get_sensors()
        # time.sleep(0.5)
        # print(time.time(), "\t", sensors.bumps_wheeldrops[0], sensors.bumps_wheeldrops[1])
        if sensors.bumps_wheeldrops[0] or sensors.bumps_wheeldrops[1]:
            print("Obstacle detected, stop disinfection")
            roomba.drive_stop()
            print("Roomba stopped")
            continueDrive = False
            stopDisinfection = True
            break
        if getRightTof() == 1 and getLeftTof() == 1: # If both sensor detects table, stop as table is aligned
            roomba.drive_stop()
            continueDrive = False
        elif getRightTof() == 1 and getLeftTof() == 0: # If right ToF detects table, move only left wheel
            roomba.drive_direct(0, DRIVE_SPEED)
        elif (getRightTof() == 0 and getLeftTof() == 1): # If left ToF detects table, move only right wheel
            roomba.drive_direct(DRIVE_SPEED, 0)
        else: # no ToF sensor detects table yet
            roomba.drive_direct(DRIVE_SPEED, DRIVE_SPEED)


def getFrontTof(): # Measure using forward ToF sensor, return 1 if Roomba is under table
    mux.disable_all()
    mux.enable_channels(1)
    ToF.start_ranging() # Write configuration bytes to initiate measurement
    time.sleep(0.05)
    distance = ToF.get_distance() # Get the result of the measurement from the sensor
    # print(distance)
    time.sleep(0.05)
    ToF.stop_ranging()
    # print("Front ToF: ", distance)
    if (distance < 1300): # may need to be adjusted
        return 1
    else:
        return 0


def getRightTof(): # Measure using right ToF sensor, return 1 if Roomba is under table
    mux.disable_all()
    mux.enable_channels(7)
    ToF.start_ranging() # Write configuration bytes to initiate measurement
    time.sleep(.005)
    distance = ToF.get_distance() # Get the result of the measurement from the sensor
    time.sleep(.005)
    ToF.stop_ranging()
        #print(distance)
    if (distance < 1300): # may need to be adjusted
        return 1
    else:
        return 0


def getLeftTof(): # Measure using left ToF sensor, return 1 if Roomba is under table
    mux.disable_all()
    mux.enable_channels(6)
    ToF.start_ranging() # Write configuration bytes to initiate measurement
    time.sleep(.005)
    distance = ToF.get_distance() # Get the result of the measurement from the sensor
    time.sleep(.005)
    ToF.stop_ranging()
        #print(distance)
    if (distance < 1300): # may need to be adjusted
        return 1
    else:
        return 0


"""
    Set arm positions
"""
def armUp():
    print("Move robot arm to up position")
    arm.write(b'\x55\x55\x02\x07')
    time.sleep(1)
    arm.write(b'\x55\x55\x05\x06\x02\x01\x00')


def armDown():
    print("Move robot arm to down position")
    arm.write(b'\x55\x55\x02\x07')
    time.sleep(1)
    arm.write(b'\x55\x55\x05\x06\x01\x01\x00')


def armStorage():
    print("Move robot arm to storage position")
    arm.write(b'\x55\x55\x02\x07')
    time.sleep(1)
    arm.write(b'\x55\x55\x05\x06\x00\x01\x00')


def tableDisinfection(roomba):
    global stopDisinfection

    time.sleep(1)
    roomba.start()
    print("Roomba starting ...")
    time.sleep(1)
    roomba.safe()
    print("Set Roomba to SAFE mode")
    time.sleep(1)

    # robotMode: 1 == wander and look for table, 2 == initial setup when detecting a table, 3 == disinfect table
    robotMode = 1 # If True, robot will wander in cleanmode to look for table. if false will disinfect table

    stopDisinfection = False # set to True if entire table has been disinfected, verified by bump and prox sensors, or if error is predicted
    lastTableDisinfectTime = -100 # Set to -10s ago

    try:
        while True:
            if robotMode == 1:
            # Set to clean mode
                armDown()
                time.sleep(5)
                roomba.clean()
                print("Set Roomba to CLEAN mode")
                
                time.sleep(0.2) # NEED TO MAKE THIOS LONGER TO ALLOW EXIT TABLE 
                for _ in range(2):
                    getFrontTof() # Throw away early measurements
                while (robotMode == 1): # Continue to look for table
                    if (getFrontTof() == 1) and (time.time() - lastTableDisinfectTime > 30): # If table detected, go to robotMode 2
                        robotMode = 2
                        print("Detected Roomba under table")
            
            elif robotMode == 2:
                # Set to safe mode
                roomba.safe()
                time.sleep(0.2)
                armUp()
            
                # STEP 1: Align robot to table using ToF sensor
                backwardShort()
                alignToTable()

                # STEP 2: Rotate ccw
                rotate90(0) # Rotate ccw after aligning with table

                # STEP 3: Move along table while alinging using ToF sensor, will stop when table leg deteced
                lookForLeg() # Move along the table until table leg is detected
                backwardShort()

                # STEP 4: Rotate clockwise to face table again
                rotate90(1) # rotate 90deg cw
                
                # STEP 5: Move back a bit to allow alignment in the next step
                backwardShort()
            
                # Next mode is to disninfect the table
                robotMode = 3
            
            elif robotMode == 3:
                while (robotMode == 3):
                    # STEP 1: alignment to table first before continue
                    alignToTable()
                    if stopDisinfection == True:
                        robotMode = 1
                        break

                    # STEP 2: Move forward until leaving table, table not detected by front ToF sensor
                    forwardUntilOutsideTable() # Should be moving forward until leaving table, but ToF not available yet
                    if stopDisinfection == True:
                        robotMode = 1
                        break
                
                    # STEP 3: Rotate to the next lane clockwise direction
                    rotate90(1)
                    forwardShort()
                    if stopDisinfection == True:
                        robotMode = 1
                        break
                    rotate90(1)
                
                    # STEP 4: alignment to table first before continue (repeat of step 1)
                    backwardShort()
                    alignToTable()
                    if stopDisinfection == True:
                        robotMode = 1
                        break

                    # Step 5: Move forward until leaving table, table not detected by front ToF sensor (repeat of step 2)
                    forwardUntilOutsideTable()
                    if stopDisinfection == True:
                        robotMode = 1
                        break
                
                    # Step 6: Rotate to the next lane counter-clockwise direction (reverse direction of step 3)
                    rotate90(0)
                    forwardShort()
                    if stopDisinfection == True:
                        robotMode = 1
                        break
                    rotate90(0)
                    
                    # Loop around to align to table again and continue to disinfect table.
                    # Disinfection will stop if entire table has been disinfected, verified by bump and prox sensors
                robotMode = 1 # Next state will be robotMode 1 to look for another table
                roomba.safe()
                time.sleep(2)
                lastTableDisinfectTime = time.time()
                stopDisinfection = False # Reset stopDisinfection value         

    except KeyboardInterrupt:
        print("Exit - from keyboard")
        roomba.drive_stop()
        roomba.safe()
        armStorage()
        time.sleep(5)


if __name__ == "__main__":
    #### Arm setup start ###
    arm = serial.Serial(port='/dev/ttyS0',
                        baudrate=9600,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        bytesize=serial.EIGHTBITS,
                        timeout=1)
    #### Arm setup end ###

    ### TOF distance sensor setup start ###
    mux = qwiic_tca9548a.QwiicTCA9548A()

    mux.disable_all()
    mux.enable_channels(1)
    ToF = qwiic_vl53l1x.QwiicVL53L1X()
    ToF.sensor_init()
    if ToF.sensor_init() == None:
        print("Sensor 1 front online!")
    ToF.set_distance_mode(2)

    mux.disable_all()
    mux.enable_channels(6)
    ToF.sensor_init()
    if ToF.sensor_init() == None:
        print("Sensor 6 right online!")
    ToF.set_distance_mode(2)

    mux.disable_all()
    mux.enable_channels(7)
    ToF.sensor_init()
    if ToF.sensor_init() == None:
        print("Sensor 7 left online!")
    ToF.set_distance_mode(2)
    ### TOF distance sensor setup end ###

    roomba = Create2("/dev/ttyUSB0", 115200)  # set up roomba
    tableDisinfection(roomba)  # start disinfection
