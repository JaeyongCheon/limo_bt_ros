#!/usr/bin/env python3
"""
고급 BT Action 노드 - 초기 위치 복귀 전용
"""

import math
from modules.base_bt_nodes import Node, Status
from nav_msgs.msg import Odometry


class ReturnToBase(Node):
    """시작 위치로 귀환 (사용자가 입력한 좌표의 반대 방향으로 이동)"""
    
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.type = "Action"
        
        # Odometry 구독
        ns = agent.ros_namespace or ""
        odom_topic = f"{ns}/odom" if ns else "/odom"
        self.current_odom = None
        self.odom_sub = self.ros.node.create_subscription(
            Odometry, odom_topic,
            lambda msg: setattr(self, 'current_odom', msg),
            10
        )
    
    def _get_yaw_from_odom(self, odom_msg):
        """Odometry에서 yaw 각도 추출"""
        q = odom_msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    async def run(self, agent, blackboard):
        # 시작 위치 가져오기
        start_position = blackboard.get('start_position')
        
        if start_position is None:
            self.ros.node.get_logger().error("❌ 시작 위치 정보 없음 - WaitForMissionCommand를 먼저 실행하세요")
            self.status = Status.FAILURE
            return self.status
        
        # 현재 위치 확인
        if self.current_odom is None:
            self.ros.node.get_logger().warn("⏳ Odometry 대기 중...", throttle_duration_sec=2.0)
            self.status = Status.RUNNING
            return self.status
        
        curr_x = self.current_odom.pose.pose.position.x
        curr_y = self.current_odom.pose.pose.position.y
        curr_yaw = self._get_yaw_from_odom(self.current_odom)
        
        start_x, start_y = start_position[0], start_position[1]
        
        # 원래 이동한 거리 계산
        moved_x = curr_x - start_x
        moved_y = curr_y - start_y
        
        # 귀환 목표: 시작 위치 (반대 방향으로 이동한 만큼 돌아감)
        return_x = start_x
        return_y = start_y
        
        # target_position에 귀환 위치 설정
        blackboard['target_position'] = (return_x, return_y, curr_yaw)
        
        self.ros.node.get_logger().info(
            f"🏠 귀환 시작\n"
            f"   시작 위치: odom({start_x:.2f}, {start_y:.2f})\n"
            f"   현재 위치: odom({curr_x:.2f}, {curr_y:.2f})\n"
            f"   이동 거리: X={moved_x:.2f}m, Y={moved_y:.2f}m\n"
            f"   귀환 목표: odom({return_x:.2f}, {return_y:.2f})"
        )
        
        self.status = Status.SUCCESS
        return self.status


