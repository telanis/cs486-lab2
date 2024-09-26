#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('blaise_safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        print("here")
        self.declare_parameter('ttc', 0.0)
        self.declare_parameter('mode', ' ')
        self.declare_parameter('student', ' ')
        
        self.threshold = self.get_parameter('ttc').value
        self.mode = self.get_parameter('mode').value
        self.student = self.get_parameter('student').value
        self.get_logger().info(f"TTC Threshold: {self.threshold}, Mode: {self.mode}, Student: {self.student}")
        self.speed = 0
        self.previous_TIME = None
        self.previous_RANGES = None
     #   self.threshold = 3.0
        # TODO: create ROS subscribers and publishers.
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scanSUB = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        if self.mode == "sim":
            print("SETTING SIM")
            self.odomSUB = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        else:
            self.odomSUB = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        
        

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        #  r is the instantaneous range measurements, and r' is the current range rate for that measurement
        
        #1. difference between prev measurement and current one
        #2. divide it by how much time has passed in between
        
        
       # ranges = np.array(scan_msg.ranges)  # ranges from the scan into array
        
        print("here")
        current_TIME = scan_msg.header.stamp.sec #current time in seconds
    
        
        if self.previous_TIME is None:  #first iteration
            self.previous_TIME = current_TIME
            self.previous_RANGES = np.array(scan_msg.ranges)
            return
            
            
        change = current_TIME - self.previous_TIME
        if change <= 0:  #avoid division by zero
            return

        current_RANGES = np.array(scan_msg.ranges)
        
        range_RATE = (current_RANGES - self.previous_RANGES) / change
        
        
        
        
        ttc = np.full_like(current_RANGES, np.inf) # array of inf 
        
        for i in range(len(current_RANGES)): # iterate from 0 to length of current_RANGES
            if range_RATE[i] < 0: #check if distace is decreasing
                ttc[i] = current_RANGES[i] / -range_RATE[i] 
        
        
        if np.any(ttc < self.threshold):
            self.get_logger().info("BRAKING NOW")
            brakeMSG = AckermannDriveStamped()
            brakeMSG.drive.speed = 0.0
            self.publisher.publish(brakeMSG)
        
        
        self.previous_TIME = current_TIME
        self.previous_RANGES = current_RANGES
        
        
        # TODO: publish command to brake
        pass

def main(args=None):
    
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
