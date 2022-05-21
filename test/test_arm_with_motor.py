import serial
import time 
from pycreate2 import Create2

roomba = Create2('/dev/ttyUSB0', 115200)
roomba.start()
roomba.safe()

arm = serial.Serial(port='/dev/ttyS0',
                    baudrate=9600,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                    timeout=1)

roomba.start_main_brush()
time.sleep(1)

# set to arm position #01
arm.write(b'\x55\x55\x02\x07')
time.sleep(1)
arm.write(b'\x55\x55\x05\x06\x01\x01\x00')
time.sleep(5)

# set to arm position #02
arm.write(b'\x55\x55\x02\x07')
time.sleep(1)
arm.write(b'\x55\x55\x05\x06\x02\x01\x00')
time.sleep(5)

# set to arm position #00
arm.write(b'\x55\x55\x02\x07')
time.sleep(1)
arm.write(b'\x55\x55\x05\x06\x00\x01\x00')
time.sleep(5)

roomba.stop_main_brush()
arm.close()
