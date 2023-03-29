import socket
import sys

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

server_address = ("192.168.0.52", 8050)
print(sys.stderr, 'starting up on %s port %s' % server_address)
sock.bind(server_address)

sock.listen(1)

while True:
    print(sys.stderr, 'waiting for a connection')
    connection, client_address = sock.accept()
    try:
        print(sys.stderr, 'connection from', client_address)

        while True:
            data = connection.recv(5).decode()
            print(data)
            print(data == 'robo1')
            print(type(data))
            print(type(data))

            if(data == "robo1"):
                print(sys.stderr, 'sending data back to the client1')
                connection.sendall("star2".encode())
                print(sys.stderr, 'sending data back to the client2')
            elif(data == "robo2"):
                connection.sendall("star3".encode())
                print(sys.stderr, 'sending star3 back to the client3')
            elif (data == "robo3"):
                print("robot is finished the maze")
            else:
                pass

    finally:
        connection.close()