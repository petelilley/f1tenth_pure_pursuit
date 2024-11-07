import rclpy
from rclpy.node import Node

from f1tenth_pure_pursuit.trapezoid_profile import TrapezoidProfile

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive

import numpy as np

ROBOT_WHEELBASE_LENGTH = 0.3

MAX_TURN_ANGLE_RAD = np.pi / 3 # TODO: Adjust based on speed

MAX_VELOCITY_MPS = 1.0
MAX_ACCELERATION_MPS2 = 1.0

class PurePursuitRunner(Node):
    def __init__(self):
        super().__init__('pure_pursuit_runner')

        self.waypoints = PoseArray()
        self.odometry = Odometry()
        self.lookahead_pose = Pose()

        self.forward_profile = TrapezoidProfile(MAX_VELOCITY_MPS, MAX_ACCELERATION_MPS2)
        # self.steering_profile = 

        self.waypoints_sub = self.create_subscription(PoseArray, 'waypoints', self.waypoints_update_listener, 10)
        self.odometry_sub = self.create_subscription(Odometry, 'ego_racecar/odom', self.odometry_update_listener, 10)

        self.lookahead_pose_pub = self.create_publisher(PoseStamped, 'lookahead_pose', 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)

        self.timer = self.create_timer(0.2, self.timer_callback)

        self.get_logger().info(f'PurePursuitRunner successfully initialized')

    def waypoints_update_listener(self, waypoints: PoseArray):
        self.waypoints = waypoints

    def odometry_update_listener(self, odometry: Odometry):
        self.odometry = odometry

    def timer_callback(self):
        ts = self.get_clock().now()

        num_waypoints = len(self.waypoints.poses)
        if num_waypoints != 0:
            closest_waypoint_index = self.find_closest_waypoint()

            max_lookahead_distance = 1.0 # TODO: Adjust based on speed

            target_waypoint_index = self.find_farthest_waypoint_in_radius(closest_waypoint_index, max_lookahead_distance)

            if target_waypoint_index == num_waypoints - 1:
                # Stop the robot
                self.forward_profile.configure_velocity(self.get_clock().now(), 0.0)
            else:
                self.forward_profile.configure_velocity(self.get_clock().now(), MAX_VELOCITY_MPS)

            self.lookahead_pose = self.waypoints.poses[target_waypoint_index]

        ps = PoseStamped()
        ps.pose = self.lookahead_pose
        ps.header.stamp = ts.to_msg()
        ps.header.frame_id = 'map'
        self.lookahead_pose_pub.publish(ps)

        steering_angle = self.calculate_steering_angle()
        
        ds = AckermannDriveStamped()
        ds.header.stamp = ts.to_msg()
        d = AckermannDrive()
        d.steering_angle = steering_angle
        d.speed = self.forward_profile.sample(ts).velocity
        ds.drive = d

        self.drive_pub.publish(ds)

    # Returns the index of the closest waypoint.
    def find_closest_waypoint(self) -> int:
        current_position = self.odometry.pose.pose.position
        
        closest_index = 0
        closest_distance = np.inf
        for i in range(len(self.waypoints.poses)):
            pose = self.waypoints.poses[i]

            waypoint_position = pose.position

            dx = waypoint_position.x - current_position.x
            dy = waypoint_position.y - current_position.y

            waypoint_distance = np.hypot(dx, dy)

            if waypoint_distance < closest_distance:
                closest_index = i
                closest_distance = waypoint_distance

        return closest_index

    # Returns the index of the closest waypoint after a starting waypoint.
    def find_farthest_waypoint_in_radius(self, start_waypoint_index, radius) -> int:
        current_position = self.odometry.pose.pose.position

        farthest_index = start_waypoint_index
        for i in range(farthest_index, len(self.waypoints.poses)):
            pose = self.waypoints.poses[i]

            waypoint_position = pose.position

            dx = waypoint_position.x - current_position.x
            dy = waypoint_position.y - current_position.y

            waypoint_distance = np.hypot(dx, dy)

            if waypoint_distance > radius:
                break

            farthest_index = i

        return farthest_index

    def calculate_steering_angle(self) -> float:
        robot_pose = self.odometry.pose.pose

        q = robot_pose.orientation
        # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        robot_yaw_angle = np.arctan2(siny_cosp, cosy_cosp);

        robot_position = robot_pose.position
        goal_position = self.lookahead_pose.position

        dx = goal_position.x - robot_position.x
        dy = goal_position.y - robot_position.y

        goal_distance = np.hypot(dx, dy)
        if goal_distance < 0.001:
            return 0.0

        goal_angle = np.arctan2(dy, dx)

        arc_point_angle = (np.pi / 2.0) - (robot_yaw_angle - goal_angle)

        arc_point_distance = np.cos(arc_point_angle) * goal_distance
        if abs(arc_point_angle) < 0.001:
            return 0.0

        # r = L^2 / 2|y|
        turn_radius = (goal_distance**2) / (2 * abs(arc_point_distance))

        angle = np.arctan(ROBOT_WHEELBASE_LENGTH / turn_radius)

        if angle > MAX_TURN_ANGLE_RAD:
            angle = MAX_TURN_ANGLE_RAD

        if arc_point_distance > 0.0:
            angle = -angle

        return angle

def main(args=None):
    rclpy.init(args=args)

    pp_runner = PurePursuitRunner()

    rclpy.spin(pp_runner)

    pp_runner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
