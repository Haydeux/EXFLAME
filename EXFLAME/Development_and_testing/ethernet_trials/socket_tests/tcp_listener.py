import socket

PORT = 1050

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

server_address = ("192.168.1.102", PORT)

try:
    client_socket.connect(server_address)
    #print("Connected")

    client_socket.sendall(b'\n')

    count = 0
    while True:
        msg = ""
        charc = ""
        while charc != "\n":
            data = client_socket.recv(1)
            charc = data.decode('utf-8')
            msg += charc
        print(count,": ", msg, sep="", end="")
        count += 1

except KeyboardInterrupt:
    pass

finally:
    client_socket.close()