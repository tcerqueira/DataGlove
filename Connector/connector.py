import socket
import serial


# Create a UDP socket at client side
serverAddressPort = ("127.0.0.1", 5433)
with socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM) as UDPClientSocket:
    # Serial connection
    serialConn = serial.Serial('COM3', 115200)

    while True:
        data = serialConn.readline()
        # print(data.decode("UTF-8"))
        UDPClientSocket.sendto(data, serverAddressPort)

    serialConn.close()
