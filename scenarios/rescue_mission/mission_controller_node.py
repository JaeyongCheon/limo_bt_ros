#!/usr/bin/env python3
# mission_controller_node.py
#
# 역할: 미션 상태머신 관리
#
# 퍼블리시:
#  - /mission_state (std_msgs/msg/String)
# 서브스크라이브:
#  - /victim_detection (std_msgs/msg/Bool)  # 조난자 발견 신호
#  - /fused_terrain (nav_msgs/msg/OccupancyGrid)  # (옵션)
#
# 파라미터:
#  - return_mode (bool) : 임무 중 복귀모드 활성화

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nav_msgs.msg import OccupancyGrid

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        self.declare_parameter('return_mode', False)
        self.state = 'IDLE'
        self.return_mode = self.get_parameter('return_mode').get_parameter_value().bool_value

        self.state_pub = self.create_publisher(String, '/mission_state', 10)
        self.victim_sub = self.create_subscription(Bool, '/victim_detection', self.victim_cb, 10)
        self.terrain_sub = self.create_subscription(OccupancyGrid, '/fused_terrain', self.terrain_cb, 1)

        self.create_timer(0.5, self.tick)
        self.get_logger().info('Mission Controller started. initial state: IDLE')

        # internal flags
        self.victim_found = False
        self.map_ready = False
        self.nav_in_progress = False
        self.search_timer = 0.0
        self.search_start_time = None

    def victim_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Victim detected!')
            self.victim_found = True

    def terrain_cb(self, msg: OccupancyGrid):
        # terrain presence indicates mapping/analysis ready
        self.map_ready = True

    def tick(self):
        # simple state machine
        if self.return_mode:
            # if a high-priority return mode set
            if self.state != 'COMPLETE':
                self.transition_to('RETURNING')
        else:
            if self.state == 'IDLE':
                if self.map_ready:
                    self.transition_to('NAVIGATING')
            elif self.state == 'NAVIGATING':
                # placeholder logic: after some time start searching
                if not self.nav_in_progress:
                    self.get_logger().info('Starting navigation (simulated)')
                    self.nav_in_progress = True
                    # simulate navigation -> after 5s switch to SEARCHING
                    self.search_start_time = self.get_clock().now().nanoseconds
                else:
                    elapsed = (self.get_clock().now().nanoseconds - self.search_start_time) / 1e9
                    if elapsed > 5.0:
                        self.transition_to('SEARCHING')
            elif self.state == 'SEARCHING':
                # searching: if victim detected -> VICTIM_FOUND
                if self.victim_found:
                    self.transition_to('VICTIM_FOUND')
                else:
                    # if searching too long -> PATH_PLANNING (replan)
                    if self.search_start_time is None:
                        self.search_start_time = self.get_clock().now().nanoseconds
                    else:
                        elapsed = (self.get_clock().now().nanoseconds - self.search_start_time) / 1e9
                        if elapsed > 20.0:
                            self.transition_to('PATH_PLANNING')
            elif self.state == 'VICTIM_FOUND':
                # once victim found, complete after publishing
                self.get_logger().info('Victim found - switching to COMPLETE')
                self.transition_to('COMPLETE')
            elif self.state == 'PATH_PLANNING':
                # we would call planner, then go to NAVIGATING
                self.transition_to('NAVIGATING')
            elif self.state == 'COMPLETE':
                # mission done
                pass
            elif self.state == 'RETURNING':
                # do returning behaviours (not implemented)
                self.transition_to('COMPLETE')

        # publish state
        s = String()
        s.data = self.state
        self.state_pub.publish(s)

    def transition_to(self, new_state: str):
        self.get_logger().info(f'Transition: {self.state} -> {new_state}')
        self.state = new_state
        # reset some flags
        if new_state == 'SEARCHING':
            self.search_start_time = self.get_clock().now().nanoseconds
            self.nav_in_progress = False

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
