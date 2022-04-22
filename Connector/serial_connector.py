import socket
import serial
import sys

def main():
    if len(sys.argv) != 2:
        exit(f"Usage: python serial_connector.py <PORT>\nExample: python3 serial_connector.py COM3")

    port = sys.argv[1];

    # Create a UDP socket at client side
    serverAddressPort = ("127.0.0.1", 5433)
    with socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM) as UDPClientSocket:
        # Serial connection
        serialConn = serial.Serial(port, 115200)
        count = 0;
        print("Running...");

        while True:
            data = serialConn.readline()
            # print(f'{count}:\t{data.decode("UTF-8")}')
            UDPClientSocket.sendto(data, serverAddressPort)
            count = count + 1

        serialConn.close()


if __name__ == '__main__':
    main()