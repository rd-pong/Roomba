import binascii
import serial
import time
import RPi.GPIO as GPIO
import qwiic_vl53l1x
import qwiic_tca9548a

roomba=serial.Serial(port='/dev/ttyUSB0',baudrate=115200)
time.sleep(0.5)
print("wait 0.5")
roomba.write(b'\x80')
time.sleep(0.5)
print("wait 0.5")
roomba.write(b'\x83')
time.sleep(0.5)
print("wait 0.5")
roomba.write(b'\x91\x00\x35\x00\x35')
print("running")
