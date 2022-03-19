import socket
import time

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

while 1:
    s.sendto("Are you Roomba?".encode(), ("100.64.15.255", 8864))
    print("sent")
    time.sleep(1)
