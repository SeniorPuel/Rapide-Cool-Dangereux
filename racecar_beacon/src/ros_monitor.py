#!/usr/bin/env python

import rospy

import rospy

import socket

import threading


from nav_msgs.msg import Odometry

from sensor_msgs.msg import LaserScan

from tf.transformations import euler_from_quaternion

from racecar_beacon.srv import MyService, MyServiceResponse
 
def quaternion_to_yaw(quat):

    # Uses TF transforms to convert a quaternion to a rotation angle around Z.

    # Usage with an Odometry message:

    #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

    return yaw

class ROSMonitor:

    def __init__(self):

        # Add your subscriber here (odom? laserscan?):

        self.sub_laser = rospy.Subscriber("/scan", LaserScan, self.scan_update)

        self.sub_odo = rospy.Subscriber("/odometry/filtered", Odometry, self.odo_update)

        # Current robot state:

        self.id = 0xFFFF

        self.pos = (0,0,0)

        self.obstacle = False

        # Params :

        self.remote_request_port = rospy.get_param("remote_request_port", 65432)

        self.pos_broadcast_port  = rospy.get_param("pos_broadcast_port", 65431)

        # Thread for RemoteRequest handling:

        self.rr_thread = threading.Thread(target=self.rr_loop)

 

        print("ROSMonitor started.")

 

    def scan_update(self, msg):

        ranges = msg

        print("Got msg from /scan: ", ranges)

 

    def odo_update(self, odo_msg):

        # Extract the position and orientation from the Odometry message

        self.position = odo_msg.pose.pose.position

        self.orientation = odo_msg.pose.pose.orientation

 

        # Convert the orientation quaternion to yaw

        yaw = quaternion_to_yaw(self.orientation)

 

        # Print the extracted information

        print("Position (X, Y, Theta): {:.2f}, {:.2f}, {:.2f}".format(self.position.x, self.position.y, yaw))

def handle_request(req):
    result = req.a + req.b
    return MyServiceResponse(result)

def server():
    rospy.init_node('my_service_server')
    s = rospy.Service('add_numbers', MyService, handle_request)
    print("ros_monitor started.")
    rospy.spin()

if __name__ == "__main__":
    server()
    
#if __name__=="__main__":
#
 #   rospy.init_node("ros_monitor")
  #  node = ROSMonitor()
   # rospy.spin()

"""def rr_loop(self):

        # Initialize your socket for remote requests here:

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as rr_socket:

            # Bind the socket to the specified port

            rr_socket.bind((HOST, self.remote_request_port))

            rr_socket.listen()

 

            print("RemoteRequest server is listening on port {}".format(self.remote_request_port))

 

            while True:

                conn, addr = rr_socket.accept()

                print("Accepted connection from", addr)

 

                # Handle remote requests here

                # Example: Send robot's current position upon request

                response = "Current Position: {}".format(self.position)

                conn.sendall(response.encode())

                conn.close()"""
