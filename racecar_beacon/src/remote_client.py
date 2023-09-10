#!/usr/bin/env python

import rospy
from racecar_beacon.srv import MyService, MyServiceRequest

def client():
    rospy.wait_for_service('add_numbers')
    try:
        add_numbers = rospy.ServiceProxy('add_numbers', MyService)
        
        # Prompt the user for two numbers
        a = int(input("Enter the first number: "))
        b = int(input("Enter the second number: "))
        
        request = MyServiceRequest(a=a, b=b)
        response = add_numbers(request)
        return response.result
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    rospy.init_node('my_service_client')
    result = client()
    rospy.loginfo(f"Result: {result}")

"""#!/usr/bin/env python

import socket

HOST = '127.0.0.1'
# This process should listen to a different port than the PositionBroadcast client.
PORT = 65432

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

 

print("Connected to server")

 

while True:
    message = input("Client > ")
    s.sendall(message.encode())
    data = s.recv(1024).decode()
    print("Server: " + data)

 

s.close()"""
