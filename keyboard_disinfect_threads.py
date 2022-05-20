'''
@author: cc2396
@date: 2022-05-05
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
import threading


# class for disinfecting keyboard
class KeyboardDisinfectThreads(object):
    def __init__(self):
        # initialize roomba
        self.roomba = Create2("/dev/ttyUSB0", 115200)
        self.roomba.start()
        time.sleep(1)
        self.roomba.safe()
        self.system_running = True

        # initialize the camera
        self.camera = cv2.VideoCapture(0)
        self.camera.set(3, 320)
        self.camera.set(4, 320)
        # setup the robotMode dictionary
        self.modes = {'no_KBD_found':1,'found_KBD':2,'steer_right':3,'steer_left':4,'clean_KBD':5}
        self.cur_mode = self.modes['no_KBD_found']

        # initialize the camera_update thread
        self.camera_thread = threading.Timer(0.5, self.camera_update)
        self.camera_thread.start() 
        self.frame = self.camera.read()[1]

        # initialize the detection model
        self.init_detection()

        # initialize the arm
        self.arm = serial.Serial(port='/dev/ttys0',
                                  baudrate=9600,
                                  parity=serial.PARITY_NONE,
                                  stopbits=serial.STOPBITS_ONE,
                                  bytesize=serial.EIGHTBITS,
                                  timeout=1)
        
    
    # detection method setup
    def init_detection(self):
        self.classNames = []
        classFile = 'coco.names'
        with open(classFile,'rt') as f:
            self.classNames = f.read().rstrip('\n').split('\n')

        configPath = 'graph.pbtxt'
        weightsPath = 'frozen_inference_graph.pb'

        self.net = cv2.dnn_DetectionModel(weightsPath,configPath)
        self.net.setInputSize(320,320)
        self.net.setInputScale(1.0/ 127.5)
        self.net.setInputMean((127.5, 127.5, 127.5))
        self.net.setInputSwapRB(True)
        # print("Detection model initialized")
    
    # move the roomba forward, speed depend on wheter the keyboard is found or not
    def move(self):
        if self.cur_mode == self.modes['no_KBD_found']:
            self.roomba.drive_direct(50,50)
        elif self.cur_mode == self.modes['found_KBD']:
            self.roomba.drive_direct(20,20)

    # stop the roomba
    def stop_move(self):
        self.roomba.drive_stop()
    
    # rotate the roomba, rotate left/right for 5 degrees or rotate 180 degrees
    def rotate(self, direction):
        angle = 0

        # rotate right
        if direction == 'right':
            self.roomba.drive_direct(-20, 20)
            while angle < 5:        
                sensors = self.roomba.get_sensors()
                angle += abs(sensors.angle)
                print("Steer right for alignment, angle rotated:", angle)
        # rotate left
        elif direction == 'left':
            self.roomba.drive_direct(20, -20)
            while angle < 5:
                sensors = self.roomba.get_sensors()
                angle += abs(sensors.angle)
                print("Steer left fot alignment, angle rotated:", angle)
        # rotate 180
        elif direction == '180':
            self.roomba.drive_direct(-20, 20)
            while angle < 180:
                sensors = self.roomba.get_sensors()
                angle += abs(sensors.angle)
                print("Rotate 180 degrees, angle rotated:", angle)

        self.roomba.drive_stop()


    # initialize the rotate_thread
    def rotate_thread(self, direction):
        rotate_thread = threading.Thread(target=self.rotate, args=(direction,))
        rotate_thread.start()
        rotate_thread.join()

    # initialize the frame, make it like an interrupt, update the frame for every 0.01 seconds 
    def camera_update(self):
        if self.system_running:
            self.frame = self.camera.read()[1]
            cv2.imshow('camera preview', self.frame)
            cv2.waitKey(1)
            self.camera_thread = threading.Timer(0.01, self.camera_update)
            self.camera_thread.start()


    # arm positions
    def armStorage(self):
        print("Move the arm to storage position")
        self.arm.write(b'\x55\x55\x05\x06\x00\x01\x00')
    
    def armSearchKeyboard(self):
        print("Move the arm to search keyboard position")
        self.arm.write(b'\x55\x55\x05\x06\x03\x01\x00')

    def armCleanKeyboardStart(self):
        print("Move the arm to cleaning keyboard start position")
        self.arm.write(b'\x55\x55\x05\x06\x04\x01\x00')

    def armCleanKeyboardEnd(self):
        print("Move the arm to cleaning keyboard end position")
        self.arm.write(b'\x55\x55\x05\x06\x05\x01\x00')
    


    # detect function, return the robotMode 
    def getObjects(self, thres, nms, draw=True, objects=[]):
        net = self.net
        frame = self.frame
        classNames = self.classNames
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

                        # steer left
                        if x_mid < 100:
                            robotMode = 4   
                        # steer right
                        elif x_mid > 220:
                            robotMode = 3   
                        # found keyboard, move slowly
                        elif object_ratio < 0.3:
                            robotMode = 2  
                        # clean keyboard
                        else:
                            robotMode = 5   
        return robotMode


    # main function
    def disinfect_keyboard(self):
        print("Starting keyboard disinfectiion...")
        # need to move the arm to search keyboard position first
        self.armSearchKeyboard()
        time.sleep(5)

        try:
            while self.system_running:
                # only run getObjects when keyboard is not cleaning
                if self.cur_mode != self.modes['clean_KBD']:
                    self.cur_mode = self.getObjects(0.5, 0.2, True, ['keyboard'])

                # no keyboard found (robotMode=1)
                if self.cur_mode == self.modes['no_KBD_found']:
                    self.move()
                    print('no keyboard found...')

                # keyboard found, move slowly (robotMode=2)
                elif self.cur_mode == self.modes['found_KBD']:
                    self.move()
                    print("found keyboard")

                # steer right (robotMode=3)
                elif self.cur_mode == self.modes['steer_right']:
                    print('steer right')
                    self.rotate_thread('right')

                # steer left (robotMode=4)
                elif self.cur_mode == self.modes['steer_left']:
                    print('steer left')
                    self.rotate_thread('left')

                # disinfect the keyboard (robotMode=5)
                elif self.cur_mode == self.modes['clean_KBD']:
                    self.stop_move()
                    print("Cleaning keyboard...")
                    '''
                    Here is the robot arm cleaning process, but having issues with the arm
                    self.armCleanKeyboardStart()
                    time.sleep(5)
                    self.armCleanKeyboardEnd()
                    time.sleep(5)
                    self.armSearchKeyboard()
                    time.sleep(5)
                    '''
                    time.sleep(10) # set 10 seconds to pretend cleaning the keyboard
                    print('Finished cleaning!')
                    print('Start rotating 180 degrees...')
                    self.rotate_thread('180')
                    self.cur_mode = self.modes['no_KBD_found']
                else:
                    print("Error: Unknown mode!")
                    self.system_running = False

        except Exception as e:
            print(e)
            self.system_running = False
            self.roomba.drive_stop()
            self.roomba.safe()
            self.armStorage()
            
        finally:
            self.camera.release()
            cv2.destroyAllWindows()


if __name__ == '__main__':
    cleanner = KeyboardDisinfectThreads()
    cleanner.disinfect_keyboard()