import socket
import serial
import sys

if len(sys.argv) != 2:
    exit(f"Usage: python connector.py <PORT>\nExample: python3 connector.py COM3")

port = sys.argv[1];

# Create a UDP socket at client side
serverAddressPort = ("127.0.0.1", 5433)
with socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM) as UDPClientSocket:
    # Serial connection
    serialConn = serial.Serial(port, 115200)
    count = 0;

    while True:
        data = serialConn.readline()
        # print(f'{count}:\t{data.decode("UTF-8")}')
        UDPClientSocket.sendto(data, serverAddressPort)
        count = count + 1

    serialConn.close()
