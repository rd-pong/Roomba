import socket
import _thread


def receiver(num, client_socket, client_addr):
    while 1:
        recv_data = client_socket.recv(1024)
        if recv_data == b"":
            print("Thread", num, "finished")
            break
        data = recv_data.decode()
        print(data)


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
    # _thread.start_new_thread(videostream, (client_addr,))
