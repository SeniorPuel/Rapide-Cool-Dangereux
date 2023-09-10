#!/usr/bin/env python

import rospy
import socket
import threading
import time
from struct import *
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from racecar_beacon.srv import MyService, MyServiceResponse

class ROSMonitor:
    def __init__(self):
        # Add your subscriber here (odom? laserscan?):
        self.sub_laser = rospy.Subscriber("/scan", LaserScan, self.scan_update)
        self.sub_odo = rospy.Subscriber("/odometry/filtered", Odometry, self.odo_update)
        
        # Current robot state:
        self.id = 10
        self.pos = [0,0,0] # x, y, theta
        self.obstacle = False
        
        # Params :
        self.remote_request_port = rospy.get_param("remote_request_port", 65432)
        #self.pos_broadcast_port  = rospy.get_param("pos_broadcast_port", 65431)
        self.pos_broadcast_port = 65431
        self.host = '127.0.0.1'
        # Thread for RemoteRequest handling:
        self.rr_thread = threading.Thread(target=self.rr_loop)
        self.pb_thread = threading.Thread(target=self.pb_loop)

        print("ROSMonitor started.")

    def scan_update(self, msg):
        # Set to Initial value
        self.obstacle = False
        # Aquire the message
        ranges = msg.ranges
        # If something is at less than 1.0m, set obstacle to true
        for value in ranges:
            if value <= 1.0: self.obstacle = False

        # Debug
        #print("Got msg from /scan: ", self.obstacle)


    def odo_update(self, odo_msg):
        # Extract the position and orientation from the Odometry message
        self.pos[0] = odo_msg.pose.pose.position.x

        self.pos[1] = odo_msg.pose.pose.position.y  
        self.pos[2] = quaternion_to_yaw(odo_msg.pose.pose.orientation)
        #print("Position (X, Y, Theta): {:.2f}, {:.2f}, {:.2f}".format(self.position.x, self.position.y, yaw))

    def pack_data(self):
        data_format = "fffI"
        data = pack(data_format, self.pos[0], self.pos[1], self.pos[2], self.id)
        print("Packed data: ", data)
        return data

    def rr_loop(self):
        # Init your socket here :
        # self.rr_socket = socket.Socket(...)
            
        self.rr_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #AF_INET for IPv4 SOCK_STREAM for TCP
        self.rr_socket.bind((self.host, self.remote_request_port))
        self.rr_socket.listen(1)

        print("Server is listening on {}:{}".format(self.host, self.remote_request_port))


        while True:  # Outer loop to keep accepting new client connections
            conn, addr = self.rr_socket.accept()
            print("Connected by", addr)
            
            while True:  # Inner loop to handle communication with the current client
                data = conn.recv(1024).decode()
                if not data:
                    break  # Exit the inner loop when the client disconnects
                print("Client: " + data)
                message = "Requête invalide"
                if data == "OBSF":
                    message = str(self.obstacle)
                    print("Message sent : {}".format(self.obstacle))
                elif data == "RPOS":
                    message = str(self.pos)
                    print("Message sent : {}".format(self.pos))
                elif data == "RBID":
                    message = str(self.id)
                    print("Message sent : {}".format(self.id))
                conn.sendall(message.encode())
            conn.close()  # Close the connection with the current client
            
        """rospy.init_node('my_service_server')
        s = rospy.Service('add_numbers', MyService, handle_request)
        print("ros_monitor started.")
        rospy.spin()

        while True:
            pass"""

    def pack_data(self):
        data_format = "fffI"
        data = pack(data_format, self.pos[0], self.pos[1], self.pos[2], self.id)
        #print("Packed data: ", data)
        return data

    def pb_loop(self):
        self.pb_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Send data to the client's broadcast or multicast address and port
        client_broadcast_address = ('<broadcast>', 65431)  # Replace with client's broadcast address or multicast group
        self.pb_socket.sendto(b'Message from server', client_broadcast_address)
        
        """
        client_socket.bind(('0.0.0.0', self.pos_broadcast_port))  # Replace 12345 with the desired port
        #self.pb_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        while True:
            data = self.pack_data()
            self.pb_socket.sendto(data, ('<broadcast>', self.pos_broadcast_port))
            time.sleep(1)"""

def quaternion_to_yaw(quat):
    (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    return yaw        

def handle_request(req):
    result = req.a + req.b
    return MyServiceResponse(result)

"""if __name__ == "__main__":
    rr_loop()"""
    
if __name__=="__main__":
    rospy.init_node("ros_monitor")
    node = ROSMonitor()
    node.rr_thread.start()
    node.pb_thread.start()
    rospy.spin()
