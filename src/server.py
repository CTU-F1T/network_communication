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

# ROS library python package
import roslib

# ROS topic python package
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
# Functions (Thread)
######################

def run_server(ip_address, port, buflen):
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
                    data = connection.recv(buflen)

                    if not data:
                        break

                    rospy.logdebug("thread: Received data of length %d." % len(data))

                    msg = message_class()
                    msg.deserialize(data)
                    pub.publish(msg)
            except Exception as e:
                rospy.logwarn(e)
            finally:
                connection.close()
                rospy.loginfo("Closed connection.")
        except:
            time.sleep(0.01)

    rospy.loginfo("Closing down the server.")


######################
# Functions
######################

def start_node(ip_address, port, topic, buflen = 128, message_type = None):
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
        message_class = topic_info[0]
    elif message_type:
        rospy.logwarn("Falling back to creating topic.")
        # Obtain message class
        try:
            message_class = roslib.message.get_message_class(message_type)
        except Exception as e:
            rospy.logerr("%s" % e)
            return

        if not message_class:
            rospy.logerr("Specified message type '%s' does not exist. Do not forget that this is case sensitive!" % message_type)
            return
    else:
        rospy.logerr("Unable to detect message type, because target topic is not currently active. Add it as another parameter to automatically create the topic.")
        return

    # Register a publisher
    pub = rospy.Publisher(topic, message_class, queue_size = 1)
    rospy.logdebug("Creating publisher to topic '%s' with message type '%s'", topic, message_class._type)

    # Create server thread
    # TODO: Check if buflen is integer
    t = threading.Thread(target = run_server, args = [ip_address, port, int(buflen)])
    t.start()

    # Function spin() simply keeps python from exiting until this node is stopped.
    rospy.spin()

    thread_run = False


if __name__ == '__main__':
    args = rospy.myargv(argv = sys.argv)

    # Slightly ugly, but efficient solution
    if len(args) == 4:
        start_node(args[1], args[2], args[3])
    elif len(args) == 5:
        start_node(args[1], args[2], args[3], args[4])
    elif len(args) > 5:
        start_node(args[1], args[2], args[3], args[4], args[5])
    else:
        print >>sys.stderr, "Error during starting up the node. Expected 3 parameters (IP address, port, topic, [buffer length, [message type]]), but received %d." % len(args)
