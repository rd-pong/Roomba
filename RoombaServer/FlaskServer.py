import _thread
import os
import socket

from flask import Flask, request, send_from_directory, abort

############# IF DEBUG ON PC, QUOTE BELOW###############
import sys
import time
# import serial
# import RPi.GPIO as GPIO

# import local pygcreate2
from pycreate2 import Create2

# from random_walk_for_ziyi import random_walk, switch
sys.path.append("/home/pi/Desktop/2021fall")
from disinfect_main_api import disinfect, switch

############# IF DEBUG ON PC, QUOTE ABOVE###############
app = Flask(__name__)
basedir = os.path.abspath(os.path.dirname(__file__))


# HTTP server
@app.route("/manu_map", methods=['GET'])
def manu_map():
    if request.method == "GET":
        if os.path.isfile(basedir + "/images/" + "map1.png"):
            return send_from_directory('images', "map1.png", as_attachment=True)
        else:
            abort(404)


@app.route("/auto_map", methods=['GET'])
def auto_map():
    if request.method == "GET":
        if os.path.isfile(basedir + "/images/" + "map1_new.png"):
            return send_from_directory('images', "map1_new.png", as_attachment=True)
        else:
            abort(404)


# TCP server
def connection():
    _thread.start_new_thread(UDPtransponder, ())
    TCP_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    TCP_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    TCP_socket.bind(("", 8866))
    TCP_socket.listen(10)
    num = 0
    print("Server is on. ")
    while 1:
        num += 1
        client_socket, client_addr = TCP_socket.accept()
        print("Connection", num, "for", client_addr)
        _thread.start_new_thread(receiver, (num, client_socket, client_addr,))


def receiver(num, client_socket, client_addr):
    while 1:
        recv_data = client_socket.recv(1024)
        if recv_data == b"":
            print("Thread", num, "finished")
            break
        data = recv_data.decode()
        print("Connection", num, client_addr, data)
        ############# IF DEBUG ON PC, QUOTE BELOW###############
        if data[:4] == "auto" or data[:4] == "manu":
            sensors = bot.get_sensors()
            batteryNow = sensors.battery_charge
            batteryMax = sensors.battery_capacity
            client_socket.send((str(int(batteryNow*100/batteryMax))[:2] + "%" + "\n").encode())
            if data[:4] == "auto":
                switch(_pause=False)
            else:
                switch(_pause=True)

        # do something
        if data[:4] == "STOP":
            bot.drive_stop()
        elif data[:4] == "FWRD":
            bot.drive_direct(4*int(data[4:]), 4*int(data[4:]))
        elif data[:4] == "BWRD":
            bot.drive_direct(-4*int(data[4:]), -4*int(data[4:]))
        elif data[:4] == "RGHT":
            bot.drive_direct(-2*int(data[4:]), 2*int(data[4:]))
        elif data[:4] == "LEFT":
            bot.drive_direct(2*int(data[4:]), -2*int(data[4:]))
        ############# IF DEBUG ON PC, QUOTE ABOVE###############


# UDP Server
def UDPtransponder():
    UDP_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    UDP_socket.bind(("", 8864))
    print("Transponder is on. ")
    while 1:
        data, (ip, port) = UDP_socket.recvfrom(1024)  # 一次接收1024字节
        if data.decode() == "Are you Roomba?":
            print("UDP broadcast from:" + ip)
            UDP_socket.sendto("I am Roomba.".encode(), (ip, 8865))


if __name__ == '__main__':
    ############# IF DEBUG ON PC, QUOTE BELOW###############
    if sys.getdefaultencoding() != 'utf-8':
        reload(sys)
        sys.setdefaultencoding('utf-8')
    sysRunning_flag = True
  
    # ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200)  # 19200

    # todo move below to 'def receiver()' ?
    bot = Create2('/dev/ttyUSB0', 115200)
    time.sleep(0.1)
    bot.start()  # equals to \x80
    bot.safe()
  
    # Launch a bash
    # _thread.start_new_thread(random_walk, (bot, ser)) # start random walk
    # _thread.start_new_thread(disinfect, (ser,)) # start disinfect
    _thread.start_new_thread(disinfect, (bot,)) # start disinfect
    ############# IF DEBUG ON PC, QUOTE ABOVE################
    _thread.start_new_thread(connection, ())
    app.run(host="0.0.0.0", port=5000, debug=False)  # blocking
    
