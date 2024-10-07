#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safet_node')
         # Initial vehicle speed
        self.speed = 0.0
         # Threshold for braking (in seconds)
        self.threshhold = 0.5
         # Angle range for detecting imminent obstacles (consider objects in front)
        self.front_angle_range = np.deg2rad(45)
         # Store previous range measurements for range rate calculation
        self.prev_ranges = None

         # Subscribe to LaserScan and Odometry
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.scan_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

         # Publisher for braking commands
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

    def odom_callback(self, odom_msg):
         # Update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
         # Extract range and angle information
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
         # Filter out NaNs and infs
        valid = np.isfinite(ranges)
        ranges = ranges[valid]
        angles = angles[valid]
         # Restrict to front angles
        front_filter = np.abs(angles) <= self.front_angle_range
        ranges = ranges[front_filter]
        angles = angles[front_filter]

         # If no valid range data, skip
        if len(ranges) == 0:
            return
        
         # Use vehicle's velocity and scan angles to calculate range rate
        range_rates = self.speed * np.cos(angles)
         # Calculate iTTC for each beam
        ittcs = np.where(range_rates < 0, ranges / -range_rates, np.inf)

         # Check if any iTTC is below threshold
        if np.any(ittcs < self.threshhold):
            self.brake()
            

    def brake(self):
         # Create and publish AckermannDriveStamped message to stop the car
        drive_msg = AckermannDriveStamped
        drive_msg.drive.speed = 0.0
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node
    rclpy.shutdown()

if __name__ == '__main__':
    main()