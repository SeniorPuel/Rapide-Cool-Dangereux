#!/usr/bin/env python

import socket
import rospy
import struct

HOST = '127.0.0.1'
# This process should listen to a different port than the RemoteRequest client.
PORT = 65431

class VehiculeTracker:
    def __init__(self):
        
        # Create a socket object
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        data, addr = client_socket.recvfrom(1024)  # Adjust buffer size as needed
        
        self.sub_Tracker = rospy.Subscriber("/odometry/filtered", ggggg, self.Unpack)
        print("Vehicle_Tracker node started.")


    # Unpacking Data:
    def Unpack(self, data):
        vehicle_id, x, y, theta = struct.unpack('fffI', data)
        print("(X, Y, Theta, ID): {:.2f}, {:.2f}, {:.2f}, {:.0f}".format(x, y, theta, vehicle_id))


if __name__ == "__main__":
    rospy.init_node("pos_poll")
    node = VehiculeTracker()
    rospy.spin()
    
    
    
