#!/usr/bin/env python3
# sensor_fusion_node.py
# ROS2 rclpy node
#
# 구독:
#  - /scan (sensor_msgs/msg/LaserScan)
#  - /depth_points (sensor_msgs/msg/PointCloud2)  # 깊이 카메라 포인트
#  - /imu (sensor_msgs/msg/Imu)
#  - /odom (nav_msgs/msg/Odometry)
#
# 퍼블리시:
#  - /fused_terrain (nav_msgs/msg/OccupancyGrid)  # 0..100 (risk scaled)
#  - /risk_map (nav_msgs/msg/OccupancyGrid)
#
# 간단 설명:
#  - 내부적으로 로컬 격자(해상도: 0.5m)를 유지.
#  - depth 카메라 가중 70%, LiDAR(Scan) 가중 30%로 높이/리스크 계산 (단순화).
#  - 출력은 0..100 (int)로 scale하여 OccupancyGrid.data에 넣음.

import rclpy
from rclpy.node import Node
import numpy as np
import math
import time

from sensor_msgs.msg import LaserScan, PointCloud2, Imu
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, PoseStamped

# PointCloud2 처리 헬퍼 (간단 디코딩)
from sensor_msgs_py import point_cloud2

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        self.declare_parameter('grid_resolution', 0.5)  # meters per cell
        self.declare_parameter('grid_size', 200)        # grid is grid_size x grid_size
        self.declare_parameter('frame_id', 'map')
        self.resolution = self.get_parameter('grid_resolution').get_parameter_value().double_value
        self.size = int(self.get_parameter('grid_size').get_parameter_value().integer_value)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # center of grid is robot origin in map frame (for simplicity)
        self.origin_x = - (self.size * self.resolution)/2.0
        self.origin_y = - (self.size * self.resolution)/2.0

        # internal grids: height estimate and weight sum
        self.height_grid = np.zeros((self.size, self.size), dtype=float)
        self.weight_grid = np.zeros((self.size, self.size), dtype=float)
        # risk grid 0..1
        self.risk_grid = np.zeros((self.size, self.size), dtype=float)
        # last robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # subscriptions
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pc_sub = self.create_subscription(PointCloud2, '/depth_points', self.pc_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 50)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 20)

        # publishers
        self.terrain_pub = self.create_publisher(OccupancyGrid, '/fused_terrain', 1)
        self.risk_pub = self.create_publisher(OccupancyGrid, '/risk_map', 1)

        # periodic publish
        self.create_timer(1.0, self.publish_maps)

        self.get_logger().info('Sensor Fusion node started (grid %dx%d, res %.2f m)' %
                               (self.size, self.size, self.resolution))

    def world_to_idx(self, x, y):
        ix = int((x - self.origin_x) / self.resolution)
        iy = int((y - self.origin_y) / self.resolution)
        if 0 <= ix < self.size and 0 <= iy < self.size:
            return ix, iy
        return None

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        # ignoring orientation for index mapping, but store yaw for local transforms if needed.
        self.robot_x = p.x
        self.robot_y = p.y
        # orientation -> yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def imu_callback(self, msg: Imu):
        # IMU could be used to weight points based on tilt; for simplicity we currently ignore.
        pass

    def scan_callback(self, msg: LaserScan):
        # integrate LaserScan distances as low-detail height/risk
        # naive: for each beam, compute endpoint coordinates and mark risk (low)
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and r > msg.range_min and r < msg.range_max:
                x = self.robot_x + r * math.cos(angle + self.robot_yaw)
                y = self.robot_y + r * math.sin(angle + self.robot_yaw)
                # for LiDAR we treat as low-detail height measurement with weight 0.3
                idx = self.world_to_idx(x, y)
                if idx:
                    ix, iy = idx
                    h = 0.0  # LaserScan doesn't give height; treat as ground-obstacle hint
                    w = 0.3
                    self.height_grid[ix, iy] = (self.height_grid[ix, iy]*self.weight_grid[ix, iy] + h*w) / (self.weight_grid[ix, iy]+w+1e-9)
                    self.weight_grid[ix, iy] += w
                    # small risk bump for obstacle
                    self.risk_grid[ix, iy] = min(1.0, self.risk_grid[ix, iy] + 0.05)
            angle += msg.angle_increment

    def pc_callback(self, msg: PointCloud2):
        # depth camera points: more accurate, weight 0.7
        # decode pointcloud
        points = point_cloud2.read_points(msg, field_names=("x","y","z"), skip_nans=True)
        for p in points:
            # points are in camera frame — assume already in map frame or sensor mounted rigidly with transforms (for simplicity assume map frame)
            x, y, z = p
            idx = self.world_to_idx(x, y)
            if idx:
                ix, iy = idx
                w = 0.7
                self.height_grid[ix, iy] = (self.height_grid[ix, iy]*self.weight_grid[ix, iy] + z*w) / (self.weight_grid[ix, iy]+w+1e-9)
                self.weight_grid[ix, iy] += w
                # risk increases with steepness / height — simple model: z > threshold increases risk
                if z > 0.5:
                    self.risk_grid[ix, iy] = min(1.0, self.risk_grid[ix, iy] + min(1.0, z/3.0))
                # near-zero z => flat => low risk

    def publish_maps(self):
        # construct OccupancyGrid for terrain (here we publish height->0..100)
        og = OccupancyGrid()
        og.header.frame_id = self.frame_id
        og.header.stamp = self.get_clock().now().to_msg()
        meta = MapMetaData()
        meta.resolution = float(self.resolution)
        meta.width = int(self.size)
        meta.height = int(self.size)
        meta.origin.position.x = float(self.origin_x)
        meta.origin.position.y = float(self.origin_y)
        og.info = meta

        # height normalized -> terrain map (not used heavily): convert heights to 0..100
        max_h = max(1.0, np.nanmax(self.height_grid))
        min_h = min(0.0, np.nanmin(self.height_grid))
        # Avoid division by zero
        denom = max(1e-6, (max_h - min_h))
        heights_norm = (self.height_grid - min_h) / denom
        heights_scaled = np.clip((heights_norm * 100.0), 0, 100).astype(np.int8)

        # risk map: 0..1 -> 0..100
        risks = np.clip((self.risk_grid * 100.0), 0, 100).astype(np.int8)

        # flatten row-major (y fastest in OccupancyGrid?) ROS uses row-major, starting at (0,0)
        og.data = list(heights_scaled.flatten(order='C'))
        self.terrain_pub.publish(og)

        rg = OccupancyGrid()
        rg.header = og.header
        rg.info = og.info
        rg.data = list(risks.flatten(order='C'))
        self.risk_pub.publish(rg)

        self.get_logger().debug('Published terrain & risk maps')

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
