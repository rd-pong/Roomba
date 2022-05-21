'''
cc2396
last modified date: 2022/04/06
status: freezing images
'''

import binascii
from cv2 import rotate
import serial
import time
import RPi.GPIO as GPIO
import qwiic_vl53l1x
import qwiic_tca9548a
import cv2

from threading import Thread
from pycreate2 import Create2

# sysRunning_flag = True


#LESSON: MUST PUT INTO SAFE MODE BEFORE STOP COMMAND 
# ser=serial.Serial(port='/dev/ttyUSB0',baudrate=115200)
# ser.flushOutput()

# Arm setup
arm=serial.Serial(port='/dev/ttyS0',
                  baudrate=9600,
                  parity=serial.PARITY_NONE,
                  stopbits=serial.STOPBITS_ONE,
                  bytesize=serial.EIGHTBITS,
                  timeout=1)



# Initialize camera
FOCAL = 220 # camera focal length
LENGTH_MOUSE = 14.3
# dist_calc = WIDTH_BALL * FOCAL / dist_rad

classNames= []
classFile = 'coco.names'
with open(classFile,'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')

configPath = 'graph.pbtxt'
weightsPath = 'frozen_inference_graph.pb'

net = cv2.dnn_DetectionModel(weightsPath,configPath)
net.setInputSize(320,320)
net.setInputScale(1.0/ 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)


"""
    rotate 5 degress
    in clockwise or counterclockwise direction
"""
def rotate5(clockwise):
    angleRotated = 0
    
    if clockwise == 0:	#ccw
        print("Rotating ccw")
        roomba.drive_direct(15, -15)
        while angleRotated < 5 :
            sensors = roomba.get_sensors()
            angleRotated += abs(sensors.angle)
            print("Angle rotate:", angleRotated)

    else:	#cw
        print("Rotating cw")
        roomba.drive_direct(-15, 15)
        while angleRotated < 5:
            sensors = roomba.get_sensors()
            angleRotated += abs(sensors.angle)
            print("Angle rotated: ", angleRotated)

    roomba.drive_stop()

def rotate180(clockwise):
    angleRotated = 0
    
    if clockwise == 0:	#ccw
        print("Rotating ccw")
        roomba.drive_direct(15, -15)
        while angleRotated < 180 :
            sensors = roomba.get_sensors()
            angleRotated += abs(sensors.angle)
            print("Angle rotate:", angleRotated)

    else:	#cw
        print("Rotating cw")
        roomba.drive_direct(-15, 15)
        while angleRotated < 180:
            sensors = roomba.get_sensors()
            angleRotated += abs(sensors.angle)
            print("Angle rotated: ", angleRotated)

    roomba.drive_stop()

def rotateCCW_10s():
    start_time = time.time()
    print("Rotating ccw for 5 sec")
    roomba.drive_direct(20, -20)

    continue_drive = True

    while continue_drive:
        if time.time() - start_time > 10:
            roomba.drive_stop()
            continue_drive = False


# Set arm positions
def armUp():
    print("Move robot arm to up position")
    arm.write(b'\x55\x55\x05\x06\x02\x01\x00')


def armDown():
    print("Move robot arm to down position")
    arm.write(b'\x55\x55\x05\x06\x01\x01\x00')


def armStorage():
    print("Move robot arm to storage position")
    arm.write(b'\x55\x55\x05\x06\x00\x01\x00')

def armSearchKeyboard():
    print("Move robot arm to search keyboard position")
    arm.write(b'\x55\x55\x05\x06\x03\x01\x00')

def armCleanKeyboardStart():
    print("Move robot arm to clean keyboard start position")
    arm.write(b'\x55\x55\x05\x06\x04\x01\x00')

def armCleanKeyboardEnd():
    print("Move robot arm to clean keyboard end position")
    arm.write(b'\x55\x55\x05\x06\x05\x01\x00')

def cleanKeybord():
    print("Cleaning ...")
    armCleanKeyboardStart()
    time.sleep(5)

    armCleanKeyboardEnd()
    time.sleep(5)

    armSearchKeyboard()
    time.sleep(5)

    # armUp()
    # time.sleep(5)

    # rotate180(1)

    # armSearchKeyboard()
    # time.sleep(5)

    print("Finish cleanig")
    global lastKeyboardDisinfectTime
    lastKeyboardDisinfectTime = time.time()
    global is_cleaning_fin
    is_cleaning_fin = True
    print("Rotate to search other keyboard")
    
    
    
def getObjects(frame,thres,nms,draw=True,objects=[]):
    classIds, confs, bbox = net.detect(frame,confThreshold=thres,nmsThreshold=nms)
    robotMode = 1
    if len(objects) == 0: 
        objects = classNames
    objectInfo = []
    
    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(),confs.flatten(),bbox):
            className = classNames[classId - 1]
            
            if className in objects:
                objectInfo.append([box, className])
                
                if (draw):
                    
                    #cv2.rectangle(frame,box,color=(0,0,255))
                    #cv2.putText(frame,className.upper(),(box[0]+10,box[1]+30),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
                    #cv2.putText(frame,str(round(confidence*100,2)),(box[0]+100,box[1]+30), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
                    
                    
                    x_mid = box[0] + box[2]/2 # Calculated and verified to be x midpoint
                    object_ratio = (box[2]*box[3])/(320*320)
                    # If x_mid < 100, turn one dir, if x_mid > 220, turn another dir
                    if x_mid < 100:
                        robotMode = 4	# Steer left
                
                    elif x_mid > 220:
                        robotMode = 3	# Steer right
                    
                    # if less than 0.4, move forward, else lean arm forward
                    elif object_ratio < 0.3:
                        robotMode = 2

                    else:
                        robotMode = 5
                    
    return frame,objectInfo,robotMode


cleaning_thread = Thread(target = cleanKeybord)
rotate180_thread = Thread(target = rotate180, args = (1,))
# rotate_cw_thread = Thread(target = rotate5, args=(0,))
# rotate_ccw_thread = Thread(target = rotate5, args=(1,))


def disinfect_keyboard(_roomba):
    global cleaning_thread
    global rotate180_thread
    global rotate_cw_thread
    global rotate_ccw_thread
    global roomba
    roomba = _roomba

    # GPIO.setmode(GPIO.BCM)   #set up GPIO pins
    #GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    #GPIO.setup(trigPin, GPIO.OUT, initial = GPIO.LOW)
    #GPIO.setup(echoPin, GPIO.IN)

    time.sleep(1)
    roomba.start()
    print("Starting keyboard disinfectiion process ...")
    time.sleep(1)
    roomba.safe()	# safe mode
    time.sleep(0.5)

    armSearchKeyboard()
    time.sleep(5)

    cap = cv2.VideoCapture(-1)
    cap.set(3, 320)
    cap.set(4, 320)

    # robotMode: 1 == wander and look for table, 2 == initial setup when detecting a table, 3 == disinfect table
    robotMode = 1 # If True, robot will wander in cleanmode to look for table. if false will disinfect table

    # stopDisinfection = False # set to True if entire table has been disinfected, verified by bump and prox sensors, or if error is predicted
    lastKeyboardDisinfectTime = -100 # Set to -10s ago

    is_cleaning = False
    is_cleaning_fin = True

    try:
        while True:
            _, frame = cap.read()

            # View camera
            if rotate180_thread.is_alive():
                result,objectInfo,robotMode = getObjects(frame,0.5,0.2,objects=["keyboard"])
            else:
                result = frame

            cv2.imshow('Output', result)
            cv2.waitKey(1)
        
            # No keyboard found
            if robotMode == 1:
                # armSearchKeyboard()
                # time.sleep(5)
                print("forward")
                roomba.drive_direct(50, 50)
        
            # Keyboard found, keep moving forward
            elif robotMode == 2:
                # armSearchKeyboard()
                # time.sleep(5)
                print("Found keyboarad, keep forward")
                roomba.drive_direct(20, 20) #wheel speed of 20
            
            # Steer right
            elif robotMode == 3:
                # armSearchKeyboard()
                # time.sleep(5)
                print("rotate cw")
                rotate5(1)
                # if not rotate_cw_thread.is_alive():
                #     try:
                #         rotate_cw_thread.start()
                #     except RuntimeError:
                #         rotate_cw_thread = Thread(target = rotate5, args=(1,))
                #         rotate_cw_thread.start()
                # time.sleep(0.5)
            
            # Steer left
            elif robotMode == 4:
                # armSearchKeyboard()
                # time.sleep(5)
                print("rotate ccw")
                rotate5(0)
                # if not rotate_ccw_thread.is_alive():
                #     try:
                #         rotate_ccw_thread.start()
                #     except RuntimeError:
                #         rotate_ccw_thread = Thread(target = rotate5, args=(0,))
                #         rotate_ccw_thread.start()
                # time.sleep(0.5)
            
            # Disinfect
            elif robotMode == 5 and time.time() - lastKeyboardDisinfectTime > 15 and is_cleaning_fin:
                is_cleaning_fin = False
                print("Near the keyboard, stop...")
                roomba.drive_stop()
                if not cleaning_thread.is_alive():
                    try:
                        cleaning_thread.start()
                    except RuntimeError:
                        cleaning_thread = Thread(target = cleanKeybord)
                        cleaning_thread.start()
                lastKeyboardDisinfectTime = time.time()


            if not cleaning_thread.is_alive():
                if not rotate180_thread.is_alive():
                    try:
                        rotate180_thread.start()
                    except RuntimeError:
                        rotate180_thread = Thread(target = rotate180, args = (1,))
                        rotate180_thread.start()
                   

                    
            
            

    except KeyboardInterrupt:
        print("Exit - from keyboard")
        roomba.drive_stop()
        roomba.safe()
        armStorage()
        time.sleep(5)

    
    # print("exit")
    # #safe mode then stop
    # time.sleep(0.2)
    # ser.write(b'\x83')#safe mode, must be in safe mode before stopping
    # time.sleep(0.2)
    # ser.write(b'\x92\x00\x00\00\00') #wheel speed of 0
    # time.sleep(0.2)
    # #stop command when we are done working
    # ser.write(b'\xAD') #stop
    # ser.close()
    # GPIO.cleanup()

if __name__ == "__main__":
    roomba = Create2("/dev/ttyUSB0", 115200)
    disinfect_keyboard(roomba)
