import binascii
import serial
import time
import qwiic_vl53l1x
import qwiic_tca9548a
import cv2
from pycreate2 import Create2
import threading

class KeyboardDisinfectThreads(object):
    def __init__(self):
        self.roomba = Create2("/dev/ttyUSB0", 115200)
        self.roomba.start()
        time.sleep(1)
        self.roomba.safe()
        self.system_running = True
        # initialize the camera
        self.camera = cv2.VideoCapture(0)
        self.camera.set(3, 320)
        self.camera.set(4, 320)
        self.modes = {'no_KBD_found':1,'found_KBD':2,'steer_right':3,'steer_left':4,'clean_KBD':5}
        self.cur_mode = self.modes['no_KBD_found']
        # initialize the thread
        self.camera_thread = threading.Timer(0.5, self.camera_update)
        self.camera_thread.start() 
        self.frame = self.camera.read()[1]
        # initialize the detection model
        self.init_detection()
    
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
        print("Detection model initialized")
    
    def move(self):
        if self.cur_mode == self.modes['no_KBD_found']:
            self.roomba.drive_direct(50,50)
        elif self.cur_mode == self.modes['found_KBD']:
            self.roomba.drive_direct(20,20)

    def stop_move(self):
        self.roomba.drive_stop()
    
    def rotate(self, direction):
        angle = 0

        # rotate right
        if direction == 'right':
            self.roomba.drive_direct(-20, 20)
            while angle < 5:        
                sensors = self.roomba.get_sensors()
                angle += abs(sensors.angle)
                print("Angle rotate:", angle)
        # rotate left
        elif direction == 'left':
            self.roomba.drive_direct(20, -20)
            while angle < 5:
                sensors = self.roomba.get_sensors()
                angle += abs(sensors.angle)
                print("Angle rotate:", angle)
        # rotate 180
        elif direction == '180':
            self.roomba.drive_direct(-20, 20)
            while angle < 180:
                sensors = self.roomba.get_sensors()
                angle += abs(sensors.angle)
                print("Angle rotate:", angle)

        self.roomba.drive_stop()





    def rotate_thread(self, direction):
        rotate_thread = threading.Thread(target=self.rotate, args=(direction,))
        rotate_thread.start()
        rotate_thread.join()

    def camera_update(self):
        if self.system_running:
            self.frame = self.camera.read()[1]
            cv2.imshow('camera preview', self.frame)
            cv2.waitKey(1)
            self.camera_thread = threading.Timer(0.01, self.camera_update)
            self.camera_thread.start()

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
                        if x_mid < 100:
                            robotMode = 4
                        elif x_mid > 220:
                            robotMode = 3
                        elif object_ratio < 0.3:
                            robotMode = 2
                        else:
                            robotMode = 5
        return robotMode

    def run(self):
        # main function
        try:
            while self.system_running:
                if self.cur_mode != self.modes['clean_KBD']:
                    self.cur_mode = self.getObjects(0.5, 0.2, True, ['keyboard'])
                if self.cur_mode == self.modes['no_KBD_found']:
                    self.move()
                    print('no keyboard found...')
                elif self.cur_mode == self.modes['found_KBD']:
                    self.move()
                    print("found keyboard")
                elif self.cur_mode == self.modes['steer_right']:
                    print('steer right')
                    self.rotate_thread('right')
                elif self.cur_mode == self.modes['steer_left']:
                    print('steer left')
                    self.rotate_thread('left')
                elif self.cur_mode == self.modes['clean_KBD']:
                    self.stop_move()
                    print("cleaning keyboard...")
                    time.sleep(10)
                    print('clean finished')
                    print('Start rotating 180...')
                    self.rotate_thread('180')
                    self.cur_mode = self.modes['no_KBD_found']
                else:
                    print("Error: unknown mode")
                    self.system_running = False
        except Exception as e:
            print(e)
            self.system_running = False
            self.roomba.drive_stop()
            self.roomba.safe()
            
        finally:
            self.camera.release()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    cleanner = KeyboardDisinfectThreads()
    cleanner.run()