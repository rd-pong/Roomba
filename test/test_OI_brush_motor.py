import serial
import time

roomba = serial.Serial(port='/dev/ttyUSB0', baudrate=115200)
time.sleep(1)
roomba.write(b'\x80')  # start
print("Roomba started")
time.sleep(1)
roomba.write(b'\x83')  # safe mode
print("Set Roomba to SAFE mode")
time.sleep(1)

try:
    print("Motor moving ...")
    roomba.write(b'\x8A\x04')  # start main brush motor only
    time.sleep(5)
    roomba.write(b'\x8A\x00')  # stop all motors
    roomba.write(b'\x83')  # safe mode
except KeyboardInterrupt:
    print("Exit - from keyboard")
    roomba.write(b'\x83')  # safe mode
    print("Set Roomba to SAFE mode")
    roomba.close()
