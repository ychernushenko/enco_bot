import socket

HOST = '128.237.250.69'
PORT = 8000

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)

client_socket, address = server_socket.accept()

while True:
        try:
                data = client_socket.recv(1024)
                print data
                if not data: break
        except KeyboardInterrupt:
                break

client_socket.close()
server_socket.close()
