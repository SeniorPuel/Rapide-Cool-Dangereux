#!/usr/bin/env python

import socket
import rospy
import struct

HOST = '127.0.0.1'
# This process should listen to a different port than the RemoteRequest client.
PORT = 65431

class VehiculeTracker:
    def __init__(self):
        print("Vehicle_Tracker")

    # Unpacking Data:
    def Unpack():
        # Create a socket object
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Bind the socket to the local host and port
        client_socket.bind(('localhost', PORT))  # You can bind to '0.0.0.0' to listen on all available interfaces

        # Listen for incoming connections
        client_socket.listen(1)  # Allow one connection at a time

        # Receive Data
        data, addr = client_socket.recvfrom(1024)  # Adjust buffer size as needed
        
        # Unpack Data
        vehicle_id, x, y, theta = struct.unpack('fffI', data)
        
        # Debug
        print("(X, Y, Theta, ID): {:.2f}, {:.2f}, {:.2f}, {:.0f}".format(x, y, theta, vehicle_id))


if __name__ == "__main__":
    rospy.init_node("Vehicle_Tracker")
    node = VehiculeTracker()
    rospy.spin()
    
    
    
    
    
    
    
    
