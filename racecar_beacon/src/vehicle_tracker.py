#!/usr/bin/env python

import socket
import threading
import struct
import rospy

PORT = 65431

client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client_socket.bind(('0.0.0.0', PORT))

print("Client listening on¸{}".format(PORT)) 

while True:
    data, addr = client_socket.recvfrom(1024) 
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
    
    
    
    
    
    
    
    
    
