import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time

class LocalNavigator(Node):
    def __init__(self):
        super().__init__('local_navigator')

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        self.front_clear = True
        self.left_clear = True
        self.right_clear = True

        # threshold distance to consider blocked
        self.block_distance = 0.5  

    def scan_callback(self, msg: LaserScan):
        # LiDAR 기준 각도 범위 계산
        ranges = msg.ranges

        # 전방  -15° ~ +15°
        front = self.get_region(ranges, -15, 15)

        # 좌측  +60° ~ +120°
        left = self.get_region(ranges, 60, 120)

        # 우측  -120° ~ -60°
        right = self.get_region(ranges, -120, -60)

        self.front_clear = min(front) > self.block_distance
        self.left_clear = min(left) > self.block_distance
        self.right_clear = min(right) > self.block_distance

    def get_region(self, ranges, start_deg, end_deg):
        total = len(ranges)
        #msg.angle_min, msg.angle_max, msg.angle_increment 이용해서 정확한 인덱스 계산
        angle_per_step = 360 / total

        # 음수 각도 → 양수 angle wrap
        start_idx = int((start_deg % 360) / angle_per_step)
        end_idx = int((end_deg % 360) / angle_per_step)

        if start_idx <= end_idx:
            return ranges[start_idx:end_idx]
        else:
            # wrap-around
            return ranges[start_idx:] + ranges[:end_idx]

    def move(self, linear=0.0, angular=0.0, duration=0.3):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular

        start = time.time()
        while time.time() - start < duration:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

        # stop
        self.cmd_pub.publish(Twist())

    # --------------------------
    # 주변 탐색 알고리즘
    # --------------------------
    def find_way_around(self):
        # 1) 전방이 열려 있으면 전진
        if self.front_clear:
            self.get_logger().info("Front clear → move forward")
            self.move(linear=0.15)
            return True

        self.get_logger().info("Front blocked. Searching...")

        # 2) 왼쪽 먼저 탐색
        if self.left_clear:
            self.get_logger().info("Left is clear → rotating left")
            self.move(angular=0.5, duration=0.8)
            return True

        # 3) 오른쪽 탐색
        if self.right_clear:
            self.get_logger().info("Right is clear → rotating right")
            self.move(angular=-0.5, duration=0.8)
            return True

        # 4) 앞/좌/우 전부 막힌 경우 → 후진 시도
        self.get_logger().info("All sides blocked → back off")
        self.move(linear=-0.1, duration=0.6)

        # 후진 후 다시 시도
        if self.front_clear or self.left_clear or self.right_clear:
            self.get_logger().info("Re-evaluating after backing up...")
            return self.find_way_around()

        # 5) 그래도 막힘 → 실패 반환
        self.get_logger().warning("Navigator: No way found")
        return False


def main(args=None):
    rclpy.init(args=args)
    node = LocalNavigator()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.find_way_around()

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
