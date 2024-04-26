# Network communication

This package is used as a way for communicating between two cars via TCP/IP.

Example:
Transfer messages from topic /scan (sensor_msgs/LaserScan) from computer A (ip: 192.168.1.2) to topic /scan_other (sensor_msgs/LaserScan) on computer B (ip: 192.168.1.3). Run:

 - B: server.py 192.168.1.3 22222 /scan_other 2048 sensor_msgs/LaserScan
 - A: client.py 192.168.1.3 22222 /scan sensor_msgs/LaserScan
