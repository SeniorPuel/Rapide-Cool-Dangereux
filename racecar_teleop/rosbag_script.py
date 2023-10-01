#!/usr/bin/env python

import rospy
import rosbag
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String  # Import the message types you want to record

def record_data():
    rospy.init_node('data_recorder', anonymous=True)
    
    # Define the topics you want to record
    topics_to_record = ['/racecar/voltage_values', '/racecar/odometry/filtered']

    # Set the bag file name and mode (e.g., 'w' for write)
    bag_filename = 'robot_data.bag'
    bag = rosbag.Bag(bag_filename, 'w')

    try:
        rospy.loginfo("Recording data...")
        #bag.write('/start_time', rospy.Time.now())  # Record a start time if needed

        # Create a callback function to write messages to the bag file
        def callback(msg):
            # Extract the x, y position, and orientation (yaw) angle
            x_position = msg.pose.pose.position.x
            y_position = msg.pose.pose.position.y
            orientation = msg.pose.pose.orientation

            # Convert orientation quaternion to yaw angle
            _, _, yaw_angle = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

            # Create Float64 messages for x, y, and yaw angle
            x_msg = Float32()
            x_msg.data = x_position

            y_msg = Float32()
            y_msg.data = y_position

            angle_msg = Float32()
            angle_msg.data = yaw_angle

            # Write the messages to the bag file
            bag.write('/racecar/x_position', x_msg)
            bag.write('/racecar/y_position', y_msg)
            bag.write('/racecar/orientation_angle', angle_msg)

        # Subscribe to the selected topics and record data
        subscribers = []
        for topic in topics_to_record:
            subscriber = rospy.Subscriber(topic, String, callback)
            subscribers.append(subscriber)

        # Keep the script running while recording
        rospy.spin()

    finally:
        rospy.loginfo("Closing the bag file...")
        #bag.write('/end_time', rospy.Time.now())  # Record an end time if needed
        bag.close()

if __name__ == '__main__':
    try:
        record_data()
    except rospy.ROSInterruptException:
        pass
