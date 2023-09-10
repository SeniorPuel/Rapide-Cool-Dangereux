#!/usr/bin/env python

import socket
import threading
import struct
import rospy

HOST = '127.0.0.1'
# This process should listen to a different port than the RemoteRequest client.
PORT = 65431

# Create a UDP socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to a specific port
client_socket.bind(('0.0.0.0', PORT))  # Replace 12345 with the desired port

# Listen for incoming packets
while True:
    data, addr = client_socket.recvfrom(1024)  # Adjust buffer size as needed
    print("Received data from {}: {}".format(addr, data.decode()))

"""client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

client_socket.bind(('0.0.0.0', PORT))  # You can bind to '0.0.0.0' to listen on all available interfaces

client_socket.listen(1)  # Allow one connection at a time

print("patate")
data, addr = client_socket.recvfrom(1024)  # Adjust buffer size as needed
print("carotte")
print("Data : ", data)
#x, y, theta, vehicle_id = struct.unpack('fffI', data)
unpacked_data = struct.unpack('fffI', data)
print("Unpacked data : ", unpacked_data)
#print("(X, Y, Theta, ID): {:.2f}, {:.2f}, {:.2f}, {:.0f}".format(x, y, theta, vehicle_id))"""
    
    
    
    
    
    
    
    
    
