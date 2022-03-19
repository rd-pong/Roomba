import socket

UDP_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
UDP_socket.bind(("", 8864))
print("Server is on. ")
while 1:
    data, (ip, port) = UDP_socket.recvfrom(1024)  # 一次接收1024字节
    if data.decode() == "Are you Roomba?":
        print(ip)
        UDP_socket.sendto("I am Roomba.".encode(), (ip, 8865))
