'''
@author: cc2396
@date: 2022-05-03
@description: disinfect keyboard with threads
@status: working but having problems with the arm
'''

import binascii
import serial
import time
import qwiic_vl53l1x
import qwiic_tca9548a
import cv2
from pycreate2 import Create2

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

classNames = []
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



# Set arm positions
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



# rotate 90 degress, 0=ccw, 1=cw
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



# rotate 180 degrees clockwise
def rotate180():
    angleRotated = 0

    print("Rotating 180 degrees")
    roomba.drive_direct(-15, 15)
    while angleRotated < 180:
        sensors = roomba.get_sensors()
        angleRotated += abs(sensors.angle)
        print("Angle rotated: ", angleRotated)

    roomba.drive_stop()




# clean keyboard
def cleanKeyboard():
    print("Cleaning...")
    # armCleanKeyboardStart()
    time.sleep(5)

    # armCleanKeyboardEnd()
    time.sleep(5)

    # armSearchKeyboard()
    time.sleep(5)

    print("Finished cleaning")




# get object
def getObjects(frame, thres, nms, draw=True, objects=[]):
    classIds, confs, bbox= net.detect(frame, confThreshold=thres, nmsThreshold=nms)
    robotMode = 1
    if len(objects) == 0:
        objects = classNames
    objectInfo = []

    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
            className = classNames[classId - 1]

            if className in objects:
                objectInfo.append([box, className])

                if (draw):

                    x_mid = box[0] + box[2]/2
                    object_ratio = (box[2]*box[3])/(320*320)
                    if x_mid < 100:
                        robotMode = 4
                    elif x_mid > 220:
                        robotMode = 3
                    elif object_ratio < 0.3:
                        robotMode = 2
                    else:
                        robotMode = 5
    return frame, objectInfo, robotMode

# disinfect the keyboard
def disinfect_keyboard(_roomba):
    global roomba
    roomba = _roomba

    time.sleep(1)
    roomba.start()
    print("Starting keyboard disinfectiion process...")
    time.sleep(1)
    roomba.safe()	# safe mode
    time.sleep(0.5)

    # armSearchKeyboard()
    time.sleep(5)

    cap = cv2.VideoCapture(-1)
    cap.set(3, 320)
    cap.set(4, 320)

    robotMode = 1
    lastKeyboardDisinfectionTime = -100
    is_cleaning = False
    clean_process = False

    try:
        while True:
            _, frame = cap.read()
            cv2.imshow('Output', frame)
            cv2.waitKey(1)


            if clean_process:
                # cv2.imshow('Output', frame)
                # cv2.waitKey(1)
                # cv2.destroyWindow('Output')
                

                if is_cleaning:
                    cleanKeyboard()
                    is_cleaning = False
                    print("Searching other keyboard")
                    rotate180()
                    clean_process = False

                    #clear the old frame
                    cap.release()
                    cap = cv2.VideoCapture(-1)
                    cap.set(3, 320)
                    cap.set(4, 320)
                    _, frame = cap.read()

            

            else:
                # _, frame = cap.read()
                result, objectInfo, robotMode = getObjects(frame, 0.5, 0.2, objects=["keyboard"])
                # cv2.imshow('Output', result)
                # cv2.waitKey(1)

                # No keyboard detected
                if robotMode == 1:
                    print("No keyboard detected")
                    roomba.drive_direct(50, 50)

                # Keyboard detected
                elif robotMode == 2:
                    print("Keyboard detected")
                    roomba.drive_direct(20, 20)

                # Steer right
                elif robotMode == 3:
                    print("Steer right")
                    rotate5(1)

                # Steer left
                elif robotMode == 4:
                    print("Steer left")
                    rotate5(0)

                # Disinfect keyboard
                elif robotMode == 5 and time.time() - lastKeyboardDisinfectionTime > 10:
                    roomba.drive_stop()
                    print("Start disinfecting keyboard...")
                    clean_process = True
                    is_cleaning = True
                
            # release cap
            # cap.release()
                         


    except KeyboardInterrupt:
        print("Exit - from keyboard")
        roomba.drive_stop()
        roomba.safe()
        # armStorage()
        time.sleep(5)


if __name__ == "__main__":
    roomba = Create2("/dev/ttyUSB0", 115200)
    disinfect_keyboard(roomba)


