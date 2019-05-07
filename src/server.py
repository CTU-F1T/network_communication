#!/usr/bin/env python
# server.py
"""Creates a server for TCP/IP communication."""
######################
# Imports & Globals
######################

# Handling arguments
import sys

# ROS python package
import rospy

# ROS topic package
import rostopic

# Network socket
import socket

# Threading
import threading

# Thread sleep
import time

# Message types
# String
from std_msgs.msg import String
#: string data


# Publishers
pub = None


# Global variables
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
thread_run = True


######################
# Callbacks
######################

def callback_obstacles(data):
    """Convert obstacles from 'obstacle_detector' to message types from 'obstacle_msgs'.

    Arguments:
    data -- structure received on a /raw_obstacles topic, defined by obstacle_detector.msg/Obstacles
    """
    msg = ObstaclesStamped()

    msg.header = data.header

    msg.obstacles = Obstacles()

    msg.obstacles.circles = [ CircleObstacle(
                                center=circle.center,
                                radius=circle.true_radius,
                                velocity=circle.velocity
                              ) for circle in data.circles ]

    msg.obstacles.segments = [ SegmentObstacle(
                                [segment.first_point, segment.last_point]
                               ) for segment in data.segments ]

    pub.publish(msg)


######################
# Functions (Thread)
######################

def run_server(ip_address, port):
    global sock
    global thread_run
    global message_class
    global pub

    # Bind socket to port
    rospy.loginfo("Starting up the server on %s:%s." % (ip_address, port))
    sock.bind((ip_address, int(port)))

    # Run as server
    sock.listen(1)

    # Run as non-blocking
    sock.setblocking(0)

    # Wait for connection
    while thread_run:
        try:
            connection, address = sock.accept()

            try:
                rospy.loginfo("Incoming connection from %s:%s." % address)

                while True:
                    data = connection.recv(128)

                    if not data:
                        break

                    rospy.logdebug("thread: Received data of length %d." % len(data))

                    msg = message_class()
                    msg.deserialize(data)
                    pub.publish(msg)
            finally:
                connection.close()
                rospy.loginfo("Closed connection.")
        except:
            time.sleep(0.01)

    rospy.loginfo("Closing down the server.")


######################
# Functions
######################

def start_node(ip_address, port, topic):
    """Starts a ROS node, registers the callbacks."""
    global thread_run
    global message_class
    global pub

    # Multiple nodes may be running, but only one should operate on each topic
    rospy.init_node("network_communication_server", anonymous = True, log_level = rospy.DEBUG)

    # Detect message type
    topic_info = rostopic.get_topic_class(topic)

    # Check if something was received and if class was detected
    if topic_info and topic_info[0]:
        # Register a publisher
        pub = rospy.Publisher(topic, topic_info[0], queue_size = 1)
        message_class = topic_info[0]
        rospy.logdebug("Creating publisher to topic '%s' with message type '%s'", topic, topic_info[0])
    else:
        rospy.logerr("Target topic is not currently active.")
        return

    # Create server thread
    t = threading.Thread(target = run_server, args = [ip_address, port])
    t.start()

    # Function spin() simply keeps python from exiting until this node is stopped.
    rospy.spin()

    thread_run = False


if __name__ == '__main__':
    args = [ a.lower() for a in rospy.myargv(argv = sys.argv) ]

    # Slightly ugly, but efficient solution
    if len(args) > 3:
        start_node(args[1], args[2], args[3])
    else:
        print >>sys.stderr, "Error during starting up the node. Expected 3 parameters (IP address, port, topic), but received %d." % len(args)
