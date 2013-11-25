import socket
import time

HOST = '128.237.250.69'
PORT = 8000

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.connect((HOST, PORT))

while True:
    server_socket.send("Hello")
    time.sleep(1)

server_socket.close()
