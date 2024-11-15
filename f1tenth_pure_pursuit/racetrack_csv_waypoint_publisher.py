import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped

import numpy as np
import csv

RACETRACK_CSV_PATH = 'Silverstone_centerline.csv'
BACKWARDS = False

class RacetrackCSVWaypointPublisher(Node):
    def __init__(self):
        super().__init__('racetrack_csv_waypoint_publisher')

        # Initialize waypoints
        self.waypoints = []

        self.load_csv()

        # Publish waypoints
        self.waypoints_pub = self.create_publisher(PoseArray, 'waypoints', 10)

        # Update timer
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info(f'RacetrackCSVWaypointPublisher successfully initialized')

    def load_csv(self):
        self.waypoints = []

        with open(RACETRACK_CSV_PATH, "r") as fp:
            reader = csv.reader(fp)
            keys = next(reader)
            for line in reader:
                pose = Pose()
                pose.position.x = float(line[0])
                pose.position.y = float(line[1])
                if BACKWARDS:
                    self.waypoints.insert(0, pose)
                else:
                    self.waypoints.append(pose)

    def timer_callback(self):
        self.publish_waypoints()

    def publish_waypoints(self):
        ts = self.get_clock().now().to_msg()

        pa = PoseArray()
        pa.poses = self.waypoints
        pa.header.stamp = ts
        pa.header.frame_id = 'map'
        self.waypoints_pub.publish(pa)








def main(args=None):
    rclpy.init(args=args)

    waypoint_publisher = RacetrackCSVWaypointPublisher()

    rclpy.spin(waypoint_publisher)

    waypoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
