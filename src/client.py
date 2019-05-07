#!/usr/bin/env python
# client.py
"""Creates a client for TCP/IP communication."""
######################
# Imports & Globals
######################

# Handling arguments
import sys

# ROS python package
import rospy

# Network socket
import socket

# Threading
import threading

# Thread sleep
import time
import StringIO

# Message types
# String
from std_msgs.msg import String
#: string data


# Global variables
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
thread_run = True
client_data = []


######################
# Callbacks
######################

def callback_client(data):
    """Receives data from a topic and sends them over network.

    Arguments:
    data -- structure received on a /client topic, defined by std_msgs.msg/String
    """
    global client_data

    a = StringIO.StringIO()
    data.serialize(a)

    client_data.append(a.getvalue())


######################
# Functions (Thread)
######################

def run_client(ip_address, port):
    global sock
    global thread_run
    global client_data

    # Bind socket to port
    rospy.loginfo("Starting up the client to %s:%s." % (ip_address, port))

    try:
        sock.connect((ip_address, int(port)))
    except:
        rospy.logerr("Connecting to server failed. Is it running?")
        rospy.loginfo("Closing down the client.")
        return

    try:
        while thread_run:
            if len(client_data) > 0:
                sock.sendall(client_data.pop(0))

            time.sleep(0.01)
    finally:
        sock.close()
        rospy.loginfo("Closing down the client.")


######################
# Functions
######################

def start_node(ip_address, port):
    """Starts a ROS node, registers the callbacks."""
    global thread_run

    # Multiple nodes may be running, but only one should operate on each topic
    rospy.init_node("network_communication_client", anonymous = True)

    # Register a callback
    rospy.Subscriber("/client", String, callback_client)

    # Create client thread
    t = threading.Thread(target = run_client, args = [ip_address, port])
    t.start()

    # Function spin() simply keeps python from exiting until this node is stopped.
    rospy.spin()

    thread_run = False


if __name__ == '__main__':
    args = [ a.lower() for a in rospy.myargv(argv = sys.argv) ]

    # Slightly ugly, but efficient solution
    if len(args) > 2:
        start_node(args[1], args[2])
    else:
        print >>sys.stderr, "Error during starting up the node. Expected 2 parameters (IP address, port), but received %d." % len(args)