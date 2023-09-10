#!/usr/bin/env python

import socket
import threading
import struct

HOST = '127.0.0.1'
# This process should listen to a different port than the RemoteRequest client.
PORT = 65431

message_format = struct.Struct('fffI')

# Create a UDP socket for listening
listen_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to a specific address and port to listen for incoming messages
listen_address = ('', PORT)  # Use an empty string for the host to bind to all available network interfaces
listen_socket.bind(listen_address)

def unpack_data(data):
    data_format = "fffI"
    unpacked_data = message_format.unpack(data)
    return unpacked_data

def listen_for_messages():
    listen_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listen_socket.bind(('', PORT))
    while True:
        data, addr = listen_socket.recvfrom(1024)
        unpacked_data = unpack_data(data)
        print(f"Received data from {addr}: {unpacked_data}")

my_thread = threading.Thread(target=listen_for_messages)

# Start the thread
my_thread.start()
