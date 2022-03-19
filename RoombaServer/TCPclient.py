import socket
import time

socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
socket.connect(("100.64.12.40", 8867))

while 1:
    socket.send("hello".encode())
    print("sent")
    time.sleep(1)
