import socket
import time


serverAddressPort   = ("127.0.0.1", 5433)

# Create a UDP socket at client side
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# Send to server using created UDP socket
for n in range(60):
    bytesToSend = str.encode(f"Spam {n}!")
    UDPClientSocket.sendto(bytesToSend, serverAddressPort)
    
bytesToSend = str.encode("Hello Server!")
UDPClientSocket.sendto(bytesToSend, serverAddressPort)

counter = 0
while True:
    msg = input("")
    bytesToSend = str.encode(f"{counter}: {msg}")
    UDPClientSocket.sendto(bytesToSend, serverAddressPort)