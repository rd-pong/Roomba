import binascii
import serial
import time
import RPi.GPIO as GPIO
import qwiic_vl53l1x
import qwiic_tca9548a

sysRunning_flag = True

#LESSON: MUST PUT INTO SAFE MODE BEFORE STOP COMMAND 
roomba=serial.Serial(port='/dev/ttyUSB0',baudrate=115200)
roomba.flushOutput()

# Arm setup
arm=serial.Serial(port='/dev/ttyS0',baudrate=9600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,timeout=1)

# TOF sensor setup
mux = qwiic_tca9548a.QwiicTCA9548A()
mux.disable_all()
mux.enable_channels(0)
ToF = qwiic_vl53l1x.QwiicVL53L1X()
ToF.sensor_init()
if (ToF.sensor_init() == None):                  # Begin returns 0 on a good init
    print("Sensor front online!\n")
ToF.set_distance_mode(2)
mux.disable_all()
mux.enable_channels(6)
ToF.sensor_init()
if (ToF.sensor_init() == None):                  # Begin returns 0 on a good init
    print("Sensor 6 right online!\n")
ToF.set_distance_mode(2)
mux.disable_all()
mux.enable_channels(7)
ToF.sensor_init()
if (ToF.sensor_init() == None):                  # Begin returns 0 on a good init
    print("Sensor 7 left online!\n")
ToF.set_distance_mode(2)

def stop():
    roomba.write(b'\x92\x00\x00\x00\x00')
    time.sleep(.1)

def rotate90(clockwise): # rotate 90 deg
    resetAngle()
    angleRotated = 0 # variable to calculated amount of rotation
    if clockwise == 0: # ccw
        print("Detected, rotate ccw")
        roomba.write(b'\x92\x00\x1E\xFF\xE2') # rotate counterclockwise
        while angleRotated < 90:
            # Send commmand to get angle rotation
            roomba.write(b'\x8E\x14') # number 20 = \x14 angle
            time.sleep(.2)
            while roomba.inWaiting() != 0: # receive message from iRobot, instead of polling maybe use interrupts/callbacks
                response = []
                for i in range(2):
                    response.append(hex(ord(roomba.read())))
                angleRotated = angleRotated + int(response[1], 16)
                print(angleRotated)
    else: # cw
        print("Detected, rotate cw")
        roomba.write(b'\x92\xFF\xE2\x00\x1E') # rotate clockwise
        while angleRotated < 90:
            # Send commmand to get angle rotation
            roomba.write(b'\x8E\x14') # number 20 = \x14 angle
            time.sleep(.2)
            while roomba.inWaiting() != 0: # receive message from iRobot, instead of polling maybe use interrupts/callbacks
                response = []
                for i in range(2):
                    temp = roomba.read()
                    #print(temp)
                    response.append(hex(ord(temp)))
                temp = int(response[1], 16)
                temp = temp ^ 0b11111111
                if temp != 255:				
                    angleRotated = angleRotated + temp + 1
                print(angleRotated)
    stop()

def forwardShort():
    global stopDisinfection
    startTime = time.time()
    roomba.write(b'\x92\x00\x2f\x00\x2f') #wheel speed of 2f
        #print("move forward for some time")
    
    move = 1 # Variable on whether we should keep moving forward
    while (move):
        if (getBumpSensor() == 1 or getProxSensorAll() == 1): # If detect a table leg, we should stop disinfecting
            stop()
            move = 0
            stopDisinfection = True
            
        elif (time.time() - startTime > 5): # If more than 5 sec
            stop()
            move = 0
     
def backwardShort():
    startTime = time.time()
    roomba.write(b'\x92\xFF\xD8\xFF\xD8') # Move wheels backward
    
    move = 1 # Variable on whether we should keep moving forward

    while (move):
        if (time.time() - startTime > 2): # If more than 2 sec
            stop()
            move = 0

def forwardUntilSensor(): # move forward until outside table based on ToF
    global stopDisinfection
    print("Move forward until outside table based on ToF")
    
    roomba.write(b'\x92\x00\x35\x00\x35') #wheel speed of 8f

    startTime2 = time.time() # Stopwatch to measure time since function first started
    
    move = 1 # Variable on whether we should keep moving forward
    underTable = 1 # Variable on whether robot is under a table or not
    for i in range(20): # throw first measurements
        getFrontTof()
    while (move):
        if (getBumpSensor() == 1 or getProxSensorAll() == 1): # If detect a table leg, we should stop disinfecting
            stop()
            move = 0
            stopDisinfection = True

        if (underTable == 1): # Table detected
            if getFrontTof() == 0: # If table no longer detected
                if (time.time() - startTime2 < 1): # If robot quit table before 1 second, we should stop disinfecting
                    stop()
                    move = 0
                    stopDisinfection = True
                underTable = 0
                startTime = time.time() # Store time since table first not detected

        else: # No table detected
            if (time.time() - startTime > 2): # Has been out of table for 2sec
                stop()
                move = 0

def lookForLeg(): # Keep moving forward, and stop when leg is detected
    roomba.write(b'\x92\x00\x35\x00\x35') #The different in left and right wheel is because of the drift caused by uneven floor
    print("Looking for leg")
    move = 1 # Variable on whether we should keep moving forward
    while (move):
        if (getBumpSensor() == 1): # If leg is detected, stop
            stop()
            move = 0
        
        # Use ToF to maintain moving along table. RightToF = 1 and Left ToF = 0
        # if (getRightTof() == 0 and getLeftTof() == 0): # If right ToF no longer detects table, slow down right wheel to steer right
        #     roomba.write(b'\x92\x00\x20\x00\x2f')
        # elif (getLeftTof() == 1 and getRightTof() == 1): # If left ToF detects table, slow down left wheel to steer left
        #     roomba.write(b'\x92\x00\x2f\x00\x20')
        # else:
        roomba.write(b'\x92\x00\x35\x00\x35')

def alignToTable(): # Align robot to table when facing to it using ToF sensor
    print("Align to table")
    global stopDisinfection
    for i in range(20):
        getRightTof()
        getLeftTof()
    move = 1 # Variable on whether we should keep moving forward
    while (move):
        if (getBumpSensor() == 1 or getProxSensorAll() == 1):
            stop()
            move = 0
            stopDisinfection = True
        if (getRightTof() == 1 and getLeftTof() == 1): # If both sensor detects table, stop as table is aligned
            stop()
            move = 0
        elif (getRightTof() == 1 and getLeftTof() == 0): # If right ToF detects table, move only left wheel
            roomba.write(b'\x92\x00\x00\x00\x2f')
        elif (getRightTof() == 0 and getLeftTof() == 1): # If left ToF detects table, move only right wheel
            roomba.write(b'\x92\x00\x2f\x00\x00')
        else: # no ToF sensor detects table yet
            roomba.write(b'\x92\x00\x2f\x00\x2f')

def getFrontTof(): # Measure using forward ToF sensor, return 1 if Roomba is under table
    mux.disable_all()
    mux.enable_channels(0)
    ToF.start_ranging() # Write configuration bytes to initiate measurement
    time.sleep(.005)
    distance = ToF.get_distance() # Get the result of the measurement from the sensor
    time.sleep(.005)
    ToF.stop_ranging()
    print("Front ToF: ", distance)
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

def getProxSensorAll(): # Get all prox sensor reading, return 1 if object detected
    roomba.write(b'\x8E\x2D') #45 = 2D all, 48 = \x30 center left, 49 = \x31 center right
    time.sleep(.2)
    while roomba.inWaiting() != 0:
        response = []
        response.append(hex(ord(roomba.read())))
        if int(response[0], 16) > 0 : # If object detected form one of the sensor
            return 1
        else:
            return 0

def getBumpSensor(): # Get bump and wheeldrop sensor, return 1 if something detected
    roomba.write(b'\x8E\x07') 
    time.sleep(.2)
    while roomba.inWaiting() != 0:
        response = []
        response.append(hex(ord(roomba.read())))
        if int(response[0], 16) > 0 : # If bump detected or robot lifted
            return 1
        else:
            return 0

def resetAngle(): # Reset angle calculation
    roomba.write(b'\x8E\x14')
    time.sleep(.2)
    while roomba.inWaiting() != 0:
        roomba.read()

def resetDistance(): # Reset distance calculation
    roomba.write(b'\x8E\x13')
    time.sleep(.2)
    while roomba.inWaiting() != 0:
        roomba.read()

# Set arm positions
def armUp():
    arm.write(b'\x55\x55\x05\x06\x02\x01\x00') # update position to ID number

def armDown():
    arm.write(b'\x55\x55\x05\x06\x01\x01\x00')

def armClean():
    arm.write(b'\x55\x55\x05\x06\x03\x01\x00')

# def armRight():
#     arm.write(b'\x55\x55\x05\x06\position\x01\x00')

# def armStop():
#     arm.write(b'\x55\x55\x02\x07')

GPIO.setmode(GPIO.BCM)   #set up GPIO pins

time.sleep(1)
roomba.write(b'\x80') #start
print("started")
time.sleep(0.2)
roomba.write(b'\x83')#safe mode
time.sleep(0.2)


# robotMode: 1 == wander and look for table, 2 == initial setup when detecting a table, 3 == disinfect table
robotMode = 1 # If True, robot will wander in cleanmode to look for table. if false will disinfect table

stopDisinfection = False # set to True if entire table has been disinfected, verified by bump and prox sensors, or if error is predicted
lastTableDisinfectTime = -10 # Set to -10s ago

try:
    while (sysRunning_flag):
        if robotMode == 1:
        # Set to clean mode
            armDown()
            time.sleep(5)
            roomba.write(b'\x87') #clean mode
            
            time.sleep(0.2) # NEED TO MAKE THIOS LONGER TO ALLOW EXIT TABLE 
            for i in range(100):
                getFrontTof() # Throw away early measurements
            while (robotMode == 1): # Continue to look for table
                if (getFrontTof() == 1) and (time.time() - lastTableDisinfectTime > 10): # If table detected, go to robotMode 2
                    robotMode = 2
        
        elif robotMode == 2:
            # Set to safe mode
            roomba.write(b'\x83')#safe mode
            time.sleep(0.2)
            armUp()
            armClean()
        
            # STEP 1: Align robot to table using ToF sensor
            backwardShort()
            alignToTable()

            # STEP 2: Rotate ccw
            print("initial ccw")
            rotate90(0) # Rotate ccw after aligning with table

            # STEP 3: Move along table while alinging using ToF sensor, will stop when table leg deteced
            print("move along table")
            lookForLeg() # Move along the table until table leg is detected
            backwardShort()

            # STEP 4: Rotate clockwise to face table again
            print("rotate cw towards table")
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
                forwardUntilSensor() # Should be moving forward until leaving table, but ToF not available yet
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
                forwardUntilSensor()
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
            lastTableDisinfectTime = time.time()
            stopDisinfection = False # Reset stopDisinfection value         

except KeyboardInterrupt:
    GPIO.cleanup() # clean up GPIO on CTRL+C exit
        #safe mode then stop
    roomba.write(b'\x83')
    time.sleep(0.2)
    #stop command when we are done working
    roomba.write(b'\xAD')
    roomba.close()

    
print("exit")
#safe mode then stop
time.sleep(0.2)
roomba.write(b'\x83')#safe mode, must be in safe mode before stopping
time.sleep(0.2)
roomba.write(b'\x92\x00\x00\00\00') #wheel speed of 0
time.sleep(0.2)
#stop command when we are done working
roomba.write(b'\xAD') #stop
roomba.close()
GPIO.cleanup()
