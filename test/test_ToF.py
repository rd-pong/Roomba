import qwiic_vl53l1x
import qwiic_tca9548a
import time

mux = qwiic_tca9548a.QwiicTCA9548A()
print(mux.connected)

mux.disable_all()
mux.enable_channels(1)
ToF = qwiic_vl53l1x.QwiicVL53L1X()
ToF.sensor_init()
if ToF.sensor_init() == None:                  # Begin returns 0 on a good init
    print("Sensor front online!")
ToF.set_distance_mode(2)

mux.disable_all()
mux.enable_channels(6)
ToF.sensor_init()
if (ToF.sensor_init() == None):                  # Begin returns 0 on a good init
    print("Sensor 6 right online!")
ToF.set_distance_mode(2)

mux.disable_all()
mux.enable_channels(7)
ToF.sensor_init()
if (ToF.sensor_init() == None):                  # Begin returns 0 on a good init
    print("Sensor 7 left online!")
ToF.set_distance_mode(2)

def getFrontTof(): # Measure using forward ToF sensor, return 1 if Roomba is under table
    mux.disable_all()
    mux.enable_channels(1)
    ToF.start_ranging() # Write configuration bytes to initiate measurement
    time.sleep(.005)
    distance = ToF.get_distance() # Get the result of the measurement from the sensor
    time.sleep(.005)
    ToF.stop_ranging()
    return distance


def getRightTof(): # Measure using right ToF sensor, return 1 if Roomba is under table
    mux.disable_all()
    mux.enable_channels(7)
    ToF.start_ranging() # Write configuration bytes to initiate measurement
    time.sleep(.005)
    distance = ToF.get_distance() # Get the result of the measurement from the sensor
    time.sleep(.005)
    ToF.stop_ranging()
    return distance


def getLeftTof(): # Measure using left ToF sensor, return 1 if Roomba is under table
    mux.disable_all()
    mux.enable_channels(6)
    ToF.start_ranging() # Write configuration bytes to initiate measurement
    time.sleep(.005)
    distance = ToF.get_distance() # Get the result of the measurement from the sensor
    time.sleep(.005)
    ToF.stop_ranging()
    return distance


for _ in range(10000):
    print("front: ", getFrontTof(), "left: ", getLeftTof(), "right: ", getRightTof())
