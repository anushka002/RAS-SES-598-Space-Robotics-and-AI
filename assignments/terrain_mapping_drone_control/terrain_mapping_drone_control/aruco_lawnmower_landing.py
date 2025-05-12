#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy as np
import cv2
from cv_bridge import CvBridge

from px4_msgs.msg import VehicleOdometry, OffboardControlMode, VehicleCommand, TrajectorySetpoint, VehicleStatus
from sensor_msgs.msg import Image, CameraInfo


class ArucoLawnmowerMission(Node):
    def __init__(self):
        super().__init__('aruco_lawnmower_mission')
        self.bridge = CvBridge()
        self.TAKEOFF_HEIGHT = 12.0
        self.LAND_HEIGHT = 0.2
        self.MARKER_SIZE = 0.8

        self.state = "WAITING_FOR_ARM"
        self.aruco_detected = False
        self.aruco_position_global = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.vehicle_odometry = None
        self.vehicle_status = None
        self.offboard_counter = 0
        self.aruco_stable_counter = 0
        self.lawnmower_started = False
        self.lawnmower_index = 0

        # Define boustrophedon path
        self.grid_origin = (0, 0)
        self.grid_width = 20
        self.grid_height = 20
        self.grid_step = 4
        self.grid_path = self.generate_lawnmower_path()

        self.qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', self.qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.qos)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', self.qos)

        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, self.qos)
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, self.qos)
        self.caminfo_sub = self.create_subscription(CameraInfo, '/drone/down_mono/camera_info', self.caminfo_callback, 10)
        self.image_sub = self.create_subscription(Image, '/drone/down_mono', self.image_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("--- Mission node initialized")

    def generate_lawnmower_path(self):
        path = []
        x0, y0 = self.grid_origin
        rows = int(self.grid_height / self.grid_step)
        cols = int(self.grid_width / self.grid_step)
        for i in range(rows):
            row = [(x0 + j * self.grid_step, y0 + i * self.grid_step) for j in range(cols)]
            if i % 2 == 1:
                row.reverse()
            path.extend(row)
        return path

    def caminfo_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info("--- Camera calibration received")

    def odom_callback(self, msg):
        self.vehicle_odometry = msg

    def status_callback(self, msg):
        self.vehicle_status = msg

    def image_callback(self, msg):
        if self.camera_matrix is None or self.vehicle_odometry is None:
            return
        if not self.lawnmower_started:  # Do not detect ArUco until survey starts
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
            corners, ids, _ = detector.detectMarkers(cv_image)

            if ids is not None and len(corners) > 0:
                marker_corners = corners[0][0]
                marker_points = np.array([
                    [-self.MARKER_SIZE / 2,  self.MARKER_SIZE / 2, 0],
                    [ self.MARKER_SIZE / 2,  self.MARKER_SIZE / 2, 0],
                    [ self.MARKER_SIZE / 2, -self.MARKER_SIZE / 2, 0],
                    [-self.MARKER_SIZE / 2, -self.MARKER_SIZE / 2, 0]
                ], dtype=np.float32)

                success, rvec, tvec = cv2.solvePnP(marker_points, marker_corners, self.camera_matrix, self.dist_coeffs)
                if not success:
                    return

                dx, dy = tvec[0][0], tvec[1][0]
                x = self.vehicle_odometry.position[0] + dy
                y = self.vehicle_odometry.position[1] + dx
                self.aruco_position_global = (x, y)
                if not self.aruco_detected:
                    self.aruco_detected = True
                    self.state = "GOTO_ARUCO"
                    self.get_logger().info(f"--- ArUco detected. Switching to GOTO_ARUCO: ({x:.2f}, {y:.2f})")
        except Exception as e:
            self.get_logger().error(f"ArUco detection error: {str(e)}")

    def publish_vehicle_command(self, cmd, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = cmd
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def publish_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def timer_callback(self):
        if self.vehicle_odometry is None:
            return

        self.publish_offboard_control_mode()

        if self.state == "WAITING_FOR_ARM":
            self.publish_setpoint(0.0, 0.0, -self.TAKEOFF_HEIGHT)
            if self.offboard_counter == 20:
                self.publish_vehicle_command(176, param1=1.0, param2=6.0)
                self.get_logger().info("--- Sent OFFBOARD command")
            if self.offboard_counter == 30:
                self.publish_vehicle_command(400, param1=1.0)
                self.get_logger().info("--- Sent ARM command")
            if self.offboard_counter > 35:
                self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            height = -self.vehicle_odometry.position[2]
            if abs(height - self.TAKEOFF_HEIGHT) > 0.3:
                self.publish_setpoint(0.0, 0.0, -self.TAKEOFF_HEIGHT)
            else:
                self.lawnmower_started = True
                self.state = "SURVEY"
                self.get_logger().info("--- Takeoff complete. Starting survey")

        elif self.state == "SURVEY":
            if self.aruco_detected:
                self.state = "GOTO_ARUCO"
                return

            if self.lawnmower_index < len(self.grid_path):
                x, y = self.grid_path[self.lawnmower_index]
                self.publish_setpoint(x, y, -self.TAKEOFF_HEIGHT)
                if math.hypot(x - self.vehicle_odometry.position[0], y - self.vehicle_odometry.position[1]) < 0.6:
                    self.lawnmower_index += 1
            else:
                self.get_logger().info("--- Survey complete. No marker found")
                self.publish_setpoint(0.0, 0.0, -self.TAKEOFF_HEIGHT)

        elif self.state == "GOTO_ARUCO":
            if self.aruco_position_global:
                x, y = self.aruco_position_global
                self.publish_setpoint(x, y, -self.TAKEOFF_HEIGHT)
                if math.hypot(x - self.vehicle_odometry.position[0], y - self.vehicle_odometry.position[1]) < 0.3:
                    self.aruco_stable_counter += 1
                    if self.aruco_stable_counter > 10:
                        self.state = "LAND"
                        self.get_logger().info("--- Stabilized. Landing...")
                else:
                    self.aruco_stable_counter = 0

        elif self.state == "LAND":
            x = self.vehicle_odometry.position[0]
            y = self.vehicle_odometry.position[1]
            self.publish_setpoint(x, y, -self.LAND_HEIGHT)
            if -self.vehicle_odometry.position[2] < 0.4:
                self.publish_vehicle_command(400, param1=0.0)
                self.get_logger().info("--- Landed")

        self.offboard_counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = ArucoLawnmowerMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("--- Mission interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

