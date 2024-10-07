import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safet_node')

         # Subscribe to LaserScan and Odometry
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.scan_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.scan_callback, 10)

         # Publisher for braking commands
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.velocity = 0.0  # Longitudinal velocity from Odometry

    def scan_callback(self, scan_msg):
         # Access the 'ranges' array from LaserScan
        ranges = np.array(scan_msg.ranges)

         # Remove inf and NaN values
        ranges = np.nan_to_num(ranges, nan=float('inf'))

         # Compute iTTC for all ranges
        iTTCs = ranges / np.abs(ranges.velocity)

         # Threshold for braking
        threshold = 2.0

         # Check if any iTTC is below the threshold
        if np.any(iTTCs < threshold):
            self.get_logger().info('Collision imminent, braking!')
            self.brake()
        else:
            self.get_logger().info('No imminent collision')

    def odom_callback(self, odom_msg):
         # Get the longitudinal velocity from Odometry
        self.velocity = odom_msg.twist.twist.linear.x

    def brake(self):
         # Create and publish AckermannDriveStamped message to stop the car
        drive_msg = AckermannDriveStamped
        drive_msg.drive.speed = 0.0
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node
    rclpy.shutdown()

if __name__ == '__main__':
    main()