import binascii
import serial
import time
import RPi.GPIO as GPIO
import qwiic_vl53l1x
import qwiic_tca9548a

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