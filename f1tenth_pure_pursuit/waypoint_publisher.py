import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped

from transforms3d import euler
from scipy.interpolate import splprep, splev
import numpy as np

RESOLUTION = 100

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

        # Initialize waypoints
        self.waypoints = []
        self.add_new_waypoint(0.0, 0.0)

        # Publish waypoints
        self.waypoints_pub = self.create_publisher(PoseArray, 'waypoints', 10)
        self.publish_waypoints()

        # Subscribe to clicked point in simulator
        self.clicked_sub = self.create_subscription(PointStamped, 'clicked_point', self.click_listener, 10)

        # Update timer
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info(f'WaypointPublisher successfully initialized')


    def add_waypoint(self, point: Point):
        self.waypoints.append(point)

    def add_new_waypoint(self, x: float, y: float):
        p = Point()
        p.x = x
        p.y = y
        self.add_waypoint(p)

    def click_listener(self, clicked: PointStamped):
        pt = clicked.point

        if (pt.z != 0.0):
            self.get_logger().error(f'Cannot add waypoint with non-zero Z component ({pt.x}, {pt.y}, {pt.z})')
            return

        self.add_waypoint(pt)
        self.get_logger().info(f'Adding waypoint ({pt.x}, {pt.y}, {pt.z})')

        # self.publish_waypoints()

    def timer_callback(self):
        self.publish_waypoints()

    def publish_waypoints(self):
        ts = self.get_clock().now().to_msg()

        poses = self.construct_smooth_curve()

        pa = PoseArray()
        pa.poses = poses
        pa.header.stamp = ts
        pa.header.frame_id = 'map'
        self.waypoints_pub.publish(pa)

    def construct_smooth_curve(self) -> list[Pose]:
        waypoint_num = len(self.waypoints)
        if waypoint_num < 1:
            return []
        elif waypoint_num == 1:
            p = Pose()
            p.position = self.waypoints[0]
            return [p]

        x = [point.x for point in self.waypoints]
        y = [point.y for point in self.waypoints]

        # Create the spline representation.
        degree = 3
        if waypoint_num == 3:
            degree = 2
        elif waypoint_num < 3:
            degree = 1

        tck, u = splprep([x, y], s=0, k=degree)

        # Calculate points.
        num = RESOLUTION * waypoint_num;
        points = splev(np.linspace(0.0, 1.0, num=num), tck)

        poses = []
        for i in range(num):
            point = Point()
            point.x = points[0][i]
            point.y = points[1][i]

            pose = Pose()
            pose.position = point

            poses.append(pose)

        return poses

def main(args=None):
    rclpy.init(args=args)

    waypoint_publisher = WaypointPublisher()

    rclpy.spin(waypoint_publisher)

    waypoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
