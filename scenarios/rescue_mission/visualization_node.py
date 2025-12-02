#!/usr/bin/env python3
# visualization_node.py
#
# RViz 시각화: SLAM 맵(유사), 3D 지형(간단), 로봇, 조난자, 3개 경로.
#
# 구독:
#  - /fused_terrain (nav_msgs/OccupancyGrid)
#  - /paths/safe, /paths/balanced, /paths/shortest (nav_msgs/Path)
#  - /victim_pose (geometry_msgs/PoseStamped)
#  - /robot_pose (geometry_msgs/PoseStamped)
#
# 퍼블리시:
#  - /visualization_marker (visualization_msgs/Marker)
#  - /visualization_marker_array (visualization_msgs/MarkerArray)

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import ColorRGBA

class VisualizationNode(Node):
    def __init__(self):
        super().__init__('rescue_visualization')
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 1)
        self.create_subscription(OccupancyGrid, '/fused_terrain', self.terrain_cb, 1)
        self.create_subscription(Path, '/paths/safe', self.safe_cb, 1)
        self.create_subscription(Path, '/paths/balanced', self.bal_cb, 1)
        self.create_subscription(Path, '/paths/shortest', self.short_cb, 1)
        self.create_subscription(PoseStamped, '/victim_pose', self.victim_cb, 1)
        self.create_subscription(PoseStamped, '/robot_pose', self.robot_cb, 1)

        # store
        self.safe_path = None
        self.bal_path = None
        self.short_path = None
        self.victim_pose = None
        self.robot_pose = None
        self.terrain = None

        self.create_timer(0.5, self.publish_markers)

    def terrain_cb(self, msg: OccupancyGrid):
        self.terrain = msg

    def safe_cb(self, msg: Path):
        self.safe_path = msg

    def bal_cb(self, msg: Path):
        self.bal_path = msg

    def short_cb(self, msg: Path):
        self.short_path = msg

    def victim_cb(self, msg: PoseStamped):
        self.victim_pose = msg

    def robot_cb(self, msg: PoseStamped):
        self.robot_pose = msg

    def publish_markers(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # robot marker
        if self.robot_pose:
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = self.robot_pose.header.frame_id
            m.ns = 'robot'
            m.id = 0
            m.type = Marker.MESH_RESOURCE
            m.action = Marker.ADD
            m.pose = self.robot_pose.pose
            m.scale.x = m.scale.y = m.scale.z = 0.6
            m.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
            # use built-in cube if mesh not available
            m.type = Marker.CUBE
            ma.markers.append(m)

        # victim marker
        if self.victim_pose:
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = self.victim_pose.header.frame_id
            m.ns = 'victim'
            m.id = 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose = self.victim_pose.pose
            m.scale.x = m.scale.y = m.scale.z = 0.5
            m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            ma.markers.append(m)

        # paths: safe-blue, balanced-green, shortest-yellow
        def path_to_marker(path, ns, mid, color, line_width=0.08):
            if not path or len(path.poses) == 0:
                return None
            m = Marker()
            m.header = path.header
            m.ns = ns
            m.id = mid
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = line_width
            m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
            for ps in path.poses:
                p = Point()
                p.x = ps.pose.position.x
                p.y = ps.pose.position.y
                p.z = ps.pose.position.z if hasattr(ps.pose.position, 'z') else 0.0
                m.points.append(p)
            return m

        if self.safe_path:
            mm = path_to_marker(self.safe_path, 'path_safe', 10, (0.0,0.0,1.0))
            if mm: ma.markers.append(mm)
        if self.bal_path:
            mm = path_to_marker(self.bal_path, 'path_balanced', 11, (0.0,1.0,0.0))
            if mm: ma.markers.append(mm)
        if self.short_path:
            mm = path_to_marker(self.short_path, 'path_shortest', 12, (1.0,1.0,0.0))
            if mm: ma.markers.append(mm)

        # publish
        if len(ma.markers) > 0:
            self.marker_pub.publish(ma)

def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
