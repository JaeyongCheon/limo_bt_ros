#!/usr/bin/env python3
"""
고급 BT Action 노드 - 구조 로봇 미션용
"""

import math
import time
from typing import List, Tuple, Dict
from collections import deque

from modules.base_bt_nodes import Node, Status
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection2DArray

try:
    from scenarios.rescue_mission.path_planner import DualPathPlanner, generate_spiral_waypoints
    from scenarios.rescue_mission.escort_mode import EscortMode
except ImportError:
    # For direct execution or when imported from bt_nodes.py
    import sys
    import os
    current_dir = os.path.dirname(os.path.abspath(__file__))
    if current_dir not in sys.path:
        sys.path.insert(0, current_dir)
    from path_planner import DualPathPlanner, generate_spiral_waypoints
    from escort_mode import EscortMode


class GenerateSpiralWaypoints(Node):
    """나선형 경로점 생성"""
    
    def __init__(self, name, agent, max_radius=5.0, angular_step=math.pi/4, radial_step=0.3):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.max_radius = max_radius
        self.angular_step = angular_step
        self.radial_step = radial_step
        self.type = "Action"
    
    async def run(self, agent, blackboard):
        # 중심점 가져오기 (blackboard 또는 현재 위치)
        center = blackboard.get('search_center')
        
        if center is None:
            # 현재 위치를 중심으로 사용
            if 'odom' in blackboard:
                odom = blackboard['odom']
                center = (odom.pose.pose.position.x, odom.pose.pose.position.y)
            else:
                center = (0.0, 0.0)
        
        # 최대 반경 (blackboard 우선)
        max_radius = blackboard.get('search_radius', self.max_radius)
        
        # 나선형 경로점 생성
        waypoints = generate_spiral_waypoints(
            center, max_radius, 
            self.angular_step, self.radial_step
        )
        
        # blackboard에 큐로 저장
        blackboard['waypoint_queue'] = deque(waypoints)
        blackboard['total_waypoints'] = len(waypoints)
        
        self.ros.node.get_logger().info(
            f"나선형 경로점 생성 완료: {len(waypoints)}개 (중심: {center}, 반경: {max_radius}m)"
        )
        
        self.status = Status.SUCCESS
        return self.status


class GetNextWaypoint(Node):
    """경로점 큐에서 다음 waypoint 추출"""
    
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.type = "Action"
    
    async def run(self, agent, blackboard):
        waypoint_queue = blackboard.get('waypoint_queue')
        
        if waypoint_queue is None or len(waypoint_queue) == 0:
            # 큐가 비었으면 SUCCESS (탐색 완료)
            blackboard['target_position'] = None
            self.status = Status.SUCCESS
            return self.status
        
        # 다음 경로점 추출
        next_waypoint = waypoint_queue.popleft()
        blackboard['target_position'] = next_waypoint
        
        remaining = len(waypoint_queue)
        total = blackboard.get('total_waypoints', 0)
        
        self.ros.node.get_logger().info(
            f"다음 경로점: ({next_waypoint[0]:.2f}, {next_waypoint[1]:.2f}) "
            f"[{total - remaining}/{total}]"
        )
        
        self.status = Status.SUCCESS
        return self.status


class GenerateRescuePaths(Node):
    """A* 기반 3가지 구조 경로 생성 (안전/균형/최단)"""
    
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.planner = DualPathPlanner()
        self.type = "Action"
    
    async def run(self, agent, blackboard):
        # 필요한 데이터 확인
        grid_map = blackboard.get('grid_map')
        victim_location = blackboard.get('victim_location')
        start_position = blackboard.get('start_position')
        
        if grid_map is None:
            self.ros.node.get_logger().warn("GridMap 데이터 없음")
            self.status = Status.FAILURE
            return self.status
        
        if victim_location is None:
            self.ros.node.get_logger().warn("조난자 위치 정보 없음")
            self.status = Status.FAILURE
            return self.status
        
        if start_position is None:
            # 현재 위치를 시작점으로
            if 'odom' in blackboard:
                odom = blackboard['odom']
                start_position = (odom.pose.pose.position.x, odom.pose.pose.position.y)
            else:
                start_position = (0.0, 0.0)
        
        # 3가지 경로 생성
        try:
            rescue_paths = self.planner.generate_rescue_paths(
                start_position, victim_location, grid_map
            )
            
            if not rescue_paths or len(rescue_paths) == 0:
                self.ros.node.get_logger().error("경로 생성 실패 - 통과 가능한 경로 없음")
                self.status = Status.FAILURE
                return self.status
            
            blackboard['rescue_paths'] = rescue_paths
            
            # 로그 출력
            self.ros.node.get_logger().info(f"구조 경로 {len(rescue_paths)}개 생성 완료:")
            for path_info in rescue_paths:
                metrics = path_info['metrics']
                self.ros.node.get_logger().info(
                    f"  - {path_info['name']}: "
                    f"거리 {metrics['distance']:.1f}m, "
                    f"시간 {metrics['time']:.0f}초, "
                    f"난이도 {metrics['difficulty']}"
                )
            
            self.status = Status.SUCCESS
            
        except Exception as e:
            self.ros.node.get_logger().error(f"경로 생성 중 오류: {e}")
            self.status = Status.FAILURE
        
        return self.status


class VisualizeResults(Node):
    """경로 시각화 (/visualization_marker_array)"""
    
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        
        # MarkerArray 퍼블리셔
        self.marker_pub = self.ros.node.create_publisher(
            MarkerArray, '/visualization_marker_array', 10
        )
        self.type = "Action"
    
    async def run(self, agent, blackboard):
        rescue_paths = blackboard.get('rescue_paths')
        victim_location = blackboard.get('victim_location')
        
        if rescue_paths is None:
            self.status = Status.FAILURE
            return self.status
        
        marker_array = MarkerArray()
        marker_id = 0
        
        # 경로 색상 (안전: 녹색, 균형: 노랑, 최단: 빨강)
        colors = [
            (0.0, 1.0, 0.0),  # 녹색
            (1.0, 1.0, 0.0),  # 노랑
            (1.0, 0.0, 0.0),  # 빨강
        ]
        
        # 각 경로 시각화
        for idx, path_info in enumerate(rescue_paths):
            path = path_info['path']
            color = colors[idx] if idx < len(colors) else (0.5, 0.5, 0.5)
            
            # 경로 라인
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.ros.node.get_clock().now().to_msg()
            marker.ns = f"rescue_path_{idx}"
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.1  # 라인 두께
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 0.8
            
            for point in path:
                p = marker.points.add() if hasattr(marker.points, 'add') else None
                if p is None:
                    from geometry_msgs.msg import Point
                    p = Point()
                    marker.points.append(p)
                p.x = point[0]
                p.y = point[1]
                p.z = 0.0
            
            marker_array.markers.append(marker)
            
            # 경로 이름 텍스트
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.ros.node.get_clock().now().to_msg()
            text_marker.ns = f"path_label_{idx}"
            text_marker.id = marker_id
            marker_id += 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # 경로 중간 지점에 텍스트 배치
            mid_point = path[len(path) // 2] if path else (0, 0)
            text_marker.pose.position.x = mid_point[0]
            text_marker.pose.position.y = mid_point[1]
            text_marker.pose.position.z = 1.0
            
            text_marker.scale.z = 0.5
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = path_info['name']
            
            marker_array.markers.append(text_marker)
        
        # 조난자 위치 마커
        if victim_location:
            victim_marker = Marker()
            victim_marker.header.frame_id = "map"
            victim_marker.header.stamp = self.ros.node.get_clock().now().to_msg()
            victim_marker.ns = "victim"
            victim_marker.id = marker_id
            victim_marker.type = Marker.SPHERE
            victim_marker.action = Marker.ADD
            victim_marker.pose.position.x = victim_location[0]
            victim_marker.pose.position.y = victim_location[1]
            victim_marker.pose.position.z = 0.5
            victim_marker.scale.x = 0.5
            victim_marker.scale.y = 0.5
            victim_marker.scale.z = 0.5
            victim_marker.color.r = 1.0
            victim_marker.color.g = 0.0
            victim_marker.color.b = 0.0
            victim_marker.color.a = 1.0
            
            marker_array.markers.append(victim_marker)
        
        # 발행
        self.marker_pub.publish(marker_array)
        
        self.ros.node.get_logger().info(
            f"경로 시각화 완료: {len(rescue_paths)}개 경로, {len(marker_array.markers)}개 마커"
        )
        
        self.status = Status.SUCCESS
        return self.status


class PublishMissionSuccess(Node):
    """미션 성공 알림"""
    
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        
        # 미션 상태 퍼블리셔
        self.status_pub = self.ros.node.create_publisher(
            String, '/mission_status', 10
        )
        self.type = "Action"
    
    async def run(self, agent, blackboard):
        victim_location = blackboard.get('victim_location')
        
        # 메시지 생성
        now = self.ros.node.get_clock().now()
        
        message = f"MISSION_SUCCESS|"
        if victim_location:
            message += f"victim_location:{victim_location[0]:.2f},{victim_location[1]:.2f}|"
        message += f"timestamp:{now.nanoseconds}"
        
        # 발행
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        
        self.ros.node.get_logger().info(f"🎉 미션 성공 알림 발행: {message}")
        
        self.status = Status.SUCCESS
        return self.status


class ReturnToBase(Node):
    """초기 위치 (0, 0)으로 귀환"""
    
    def __init__(self, name, agent, base_position=(0.0, 0.0)):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.base_position = base_position
        self.type = "Action"
    
    async def run(self, agent, blackboard):
        # 귀환 위치 설정 (blackboard 우선)
        base = blackboard.get('base_position', self.base_position)
        
        # target_position에 귀환 위치 설정
        blackboard['target_position'] = base
        
        self.ros.node.get_logger().info(
            f"🏠 귀환 위치 설정: ({base[0]:.2f}, {base[1]:.2f})"
        )
        
        self.status = Status.SUCCESS
        return self.status


class WaitForRescueTeam(Node):
    """구조대 준비 신호 대기"""
    
    def __init__(self, name, agent, timeout=30.0):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.timeout = timeout
        self.start_time = None
        self.type = "Action"
    
    async def run(self, agent, blackboard):
        # 시작 시간 기록
        if self.start_time is None:
            self.start_time = self.ros.node.get_clock().now()
            self.ros.node.get_logger().info("구조대 준비 신호 대기 중...")
        
        # 구조대 준비 확인
        rescue_team_ready = blackboard.get('rescue_team_ready', False)
        
        if rescue_team_ready:
            self.ros.node.get_logger().info("✅ 구조대 준비 완료!")
            self.start_time = None
            self.status = Status.SUCCESS
            return self.status
        
        # 타임아웃 체크
        elapsed = (self.ros.node.get_clock().now() - self.start_time).nanoseconds / 1e9
        timeout = blackboard.get('rescue_team_timeout', self.timeout)
        
        if elapsed >= timeout:
            self.ros.node.get_logger().warn(f"⏰ 구조대 대기 타임아웃 ({timeout}초)")
            self.start_time = None
            # 타임아웃되어도 SUCCESS (계속 진행)
            blackboard['rescue_team_ready'] = True
            self.status = Status.SUCCESS
            return self.status
        
        # 계속 대기
        self.status = Status.RUNNING
        return self.status


class EscortToVictim(Node):
    """구조대와 함께 조난자 위치로 이동"""
    
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.escort_mode = None
        self.escort_info = None
        self.type = "Action"
    
    async def run(self, agent, blackboard):
        # 초기화
        if self.escort_mode is None:
            self.escort_mode = EscortMode(logger=self.ros.node.get_logger())
        
        # 필요한 데이터
        rescue_paths = blackboard.get('rescue_paths')
        grid_map = blackboard.get('grid_map')
        victim_location = blackboard.get('victim_location')
        
        if not rescue_paths or not grid_map or not victim_location:
            self.ros.node.get_logger().error("동반 안내에 필요한 데이터 부족")
            self.status = Status.FAILURE
            return self.status
        
        # 안전 경로 선택 (첫 번째 경로)
        safe_path_info = rescue_paths[0]
        safe_path = safe_path_info['path']
        
        # 지형 데이터 추출
        planner = DualPathPlanner()
        terrain_data = planner._extract_terrain_data(grid_map)
        
        # 동반 안내 시작 (첫 실행 시)
        if self.escort_info is None:
            self.escort_info = self.escort_mode.start_escort(
                safe_path, terrain_data, victim_location
            )
            blackboard['escort_info'] = self.escort_info
        
        # 현재 위치
        current_pos = (0.0, 0.0)
        if 'odom' in blackboard:
            odom = blackboard['odom']
            current_pos = (odom.pose.pose.position.x, odom.pose.pose.position.y)
        
        # 다음 경로점 가져오기
        next_waypoint = self.escort_mode.get_next_waypoint(current_pos, self.escort_info)
        
        if next_waypoint is None:
            # 안내 완료
            self.ros.node.get_logger().info("🎯 동반 안내 완료!")
            self.escort_mode = None
            self.escort_info = None
            self.status = Status.SUCCESS
            return self.status
        
        # 경로점을 target_position에 설정
        blackboard['target_position'] = next_waypoint
        
        # 경로점 도달 확인 (간단히 거리로 판단)
        dist = math.sqrt(
            (current_pos[0] - next_waypoint[0])**2 + 
            (current_pos[1] - next_waypoint[1])**2
        )
        
        if dist < 0.5:  # 0.5m 이내 도달
            self.escort_mode.advance_waypoint()
        
        self.status = Status.RUNNING
        return self.status


class CheckTeamFollowing(Node):
    """구조대가 따라오는지 확인 (Condition 역할이지만 Action으로 구현)"""
    
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.type = "Action"
    
    async def run(self, agent, blackboard):
        # 구조대 추적 시스템이 없으므로 항상 SUCCESS
        # 실제로는 구조대 위치를 추적하여 판단
        
        self.ros.node.get_logger().info("구조대 추종 확인 (시뮬레이션)")
        
        self.status = Status.SUCCESS
        return self.status


class WarnDangerZone(Node):
    """위험 구간 접근 시 경고"""
    
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.type = "Action"
    
    async def run(self, agent, blackboard):
        escort_info = blackboard.get('escort_info')
        
        if escort_info is None:
            self.status = Status.SUCCESS
            return self.status
        
        # 위험 구간 정보
        danger_zones = escort_info.get('danger_zones', [])
        warnings = escort_info.get('warnings', [])
        
        if warnings:
            for warning in warnings:
                self.ros.node.get_logger().warn(f"⚠️ {warning}")
        
        self.ros.node.get_logger().info(
            f"위험 구간 분석 완료: {len(danger_zones)}개 구간 식별"
        )
        
        self.status = Status.SUCCESS
        return self.status


class AnnounceArrival(Node):
    """조난자 위치 도착 알림"""
    
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        
        # 알림 퍼블리셔
        self.announcement_pub = self.ros.node.create_publisher(
            String, '/mission_announcement', 10
        )
        self.type = "Action"
    
    async def run(self, agent, blackboard):
        victim_location = blackboard.get('victim_location')
        
        message = "조난자 위치에 도착했습니다!"
        if victim_location:
            message += f" 위치: ({victim_location[0]:.2f}, {victim_location[1]:.2f})"
        
        # 로그 출력
        self.ros.node.get_logger().info(f"📢 {message}")
        
        # 토픽 발행
        msg = String()
        msg.data = message
        self.announcement_pub.publish(msg)
        
        self.status = Status.SUCCESS
        return self.status

