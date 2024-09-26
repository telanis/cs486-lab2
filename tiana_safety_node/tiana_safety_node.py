#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import math
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('tiana_safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """

        #self.declare_parameter('ttc', 0.0)
        #self.declare_parameter('mode', ' ')
        #self.declare_parameter('student', ' ')

        #self.threshold = self.get_parameter('ttc').value
        #self.mode = self.get_parameter('mode').value
        #self.student = self.get_parameter('student').value
        #self.get_logger().info(f"TTC: {self.threshold}, Mode: {self.mode}, Student: {self.student}")


        self.declare_parameter('ttc', 0.0)
        self.declare_parameter('mode', ' ')
        self.declare_parameter('student', ' ')

        self.threshold = self.get_parameter('ttc').value
        self.mode = self.get_parameter('mode').value
        self.student = self.get_parameter('student').value
        self.get_logger().info(f"TTC Threshold: {self.threshold}, Mode: {self.mode}, Student: {self.student}")

        self.threshold = 0.5
        self.speed = 0.
        self.previous_TIME = None
        self.previous_RANGES = None
        # TODO: create ROS subscribers and publishers.
        # TODO: create /scan subscriber
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        #self.scan_sub #prevent unused variable warning
        # TODO: create /drive publisher
        self.ack_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        #if self.mode != "sim":
            #self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        #else:
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)


    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x
        # tests
        #self.get_logger().info("Testing odom callback")

    def scan_callback(self, scan_msg):


        # tests
        #self.get_logger().info("Testing scan callback")
        curr_speed = self.speed

        ranges = np.array(scan_msg.ranges)

        #self.get_logger().info(str(ranges)) # testing

        ittc = np.full_like(ranges, np.inf)

        for i in range(len(ranges)):
            if self.speed > 0.5:
                ittc[i] = ranges[i] / (curr_speed * math.cos(scan_msg.angle_increment * i))

        self.get_logger().info(str(ittc))

        if np.any(ittc < self.threshold):
            self.get_logger().info("SLOW DOWN PLS")
            ackmsg = AckermannDriveStamped()
            ackmsg.drive.speed = 0.0
            self.ack_pub.publish(ackmsg)



        pass



def main(args=None):
    rclpy.init(args=args)
    tiana_safety_node = SafetyNode()
    rclpy.spin(tiana_safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tiana_safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()