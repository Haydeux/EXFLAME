import socket

PORT = 8080
hello = b"Hello from client\n"

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

server_address = ("192.168.1.102", PORT)

try:
    client_socket.connect(server_address)
    print("Connection established")

    client_socket.sendall(hello)
    print("Hello message sent")

    data = client_socket.recv(1024)
    print(data.decode('utf-8'))

finally:
    client_socket.close()