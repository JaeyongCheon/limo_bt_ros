#!/usr/bin/env python3
"""
기본 BT Action 노드 - 주행 팀원 2 구현
구조 로봇 미션의 기본 액션 노드 10개

참고: PROJECT_RESCUE_ROBOT.md 498-548줄
"""

import re
import math
import time
from modules.base_bt_nodes import Node, Status
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from modules.base_bt_nodes_ros import ActionWithROSAction
from geometry_msgs.msg import PoseStamped


class LogMessage(Node):
    """
    로그 메시지 출력
    
    기능:
    - ROS2 logger를 통해 메시지 출력
    - INFO, WARN, ERROR 레벨 지원
    - BT 실행 중 디버깅 및 상태 추적
    
    사용 예시:
    <LogMessage message="미션 시작" level="INFO"/>
    """
    
    def __init__(self, name, agent, message="", level="INFO"):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.message = message
        self.level = level.upper()
        self.type = "Action"
    
    async def run(self, agent, blackboard):
        # blackboard에서 동적 메시지 가져오기 (있으면)
        msg = blackboard.get('log_message', self.message)
        
        # 로그 레벨에 따라 출력
        if self.level == "INFO":
            self.ros.node.get_logger().info(msg)
        elif self.level == "WARN":
            self.ros.node.get_logger().warn(msg)
        elif self.level == "ERROR":
            self.ros.node.get_logger().error(msg)
        else:
            self.ros.node.get_logger().info(msg)
        
        self.status = Status.SUCCESS
        return self.status


class WaitForMissionCommand(Node):
    """
    미션 명령 대기 및 수신 (터미널 입력 방식)
    
    기능:
    - 터미널에서 x, y 좌표를 직접 입력받음
    - 입력받은 좌표를 target_position에 저장
    - 비동기 방식으로 입력 대기
    
    Blackboard 출력:
    - 'target_position': (x, y, 0.0) 목표 좌표 (odom 프레임)
    """
    
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.type = "Action"
        self.coords_received = False
        self.input_requested = False
    
    async def run(self, agent, blackboard):
        if self.coords_received:
            # 이미 좌표를 받았으면 SUCCESS
            self.status = Status.SUCCESS
            return self.status
        
        if not self.input_requested:
            # 최초 1회만 입력 요청
            self.input_requested = True
            self.ros.node.get_logger().info("")
            self.ros.node.get_logger().info("=" * 60)
            self.ros.node.get_logger().info("📍 목표 좌표 입력 대기중... (odom 프레임 기준)")
            self.ros.node.get_logger().info("🎯 목표 좌표를 입력하세요")
            self.ros.node.get_logger().info("=" * 60)
            
            try:
                # 비차단 방식으로 입력 받기
                x_str = input("X 좌표 (미터): ")
                y_str = input("Y 좌표 (미터): ")
                
                x = float(x_str)
                y = float(y_str)
                
                # 좌표를 target_position으로 저장 (odom 프레임 기준, yaw=0)
                target_position = (x, y, 0.0)
                blackboard['target_position'] = target_position
                self.coords_received = True
                
                self.ros.node.get_logger().info("")
                self.ros.node.get_logger().info("=" * 60)
                self.ros.node.get_logger().info(
                    f"✅ 목표 좌표 설정 완료: x={x:.2f}m, y={y:.2f}m (odom 프레임)"
                )
                self.ros.node.get_logger().info("=" * 60)
                self.ros.node.get_logger().info("")
                
                self.status = Status.SUCCESS
                return self.status
                
            except ValueError:
                self.ros.node.get_logger().error("❌ 잘못된 입력 형식입니다. 숫자를 입력하세요.")
                self.input_requested = False
                self.status = Status.RUNNING
                return self.status
            except Exception as e:
                self.ros.node.get_logger().error(f"❌ 입력 오류: {e}")
                self.input_requested = False
                self.status = Status.RUNNING
                return self.status
        
        # 입력 대기 중
        self.status = Status.RUNNING
        return self.status
    
    def reset(self):
        """노드 리셋"""
        self.coords_received = False
        self.input_requested = False


class ParseTargetCommand(Node):
    """
    목표 좌표 검증 (더 이상 파싱 불필요 - 직접 입력 방식)
    
    기능:
    - target_position 존재 확인
    - 좌표 유효성 검증
    
    Blackboard 입력:
    - 'target_position': (x, y, yaw) 목표 좌표
    
    Blackboard 출력:
    - 검증 성공 시 그대로 유지
    """
    
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.type = "Action"
    
    async def run(self, agent, blackboard):
        target_position = blackboard.get('target_position')
        
        if target_position is None:
            self.ros.node.get_logger().error("❌ 목표 좌표가 설정되지 않았습니다")
            self.status = Status.FAILURE
            return self.status
        
        # 좌표 유효성 검증
        if len(target_position) < 2:
            self.ros.node.get_logger().error("❌ 잘못된 좌표 형식")
            self.status = Status.FAILURE
            return self.status
        
        x, y = target_position[0], target_position[1]
        yaw = target_position[2] if len(target_position) > 2 else 0.0
        
        self.ros.node.get_logger().info(
            f"✅ 목표 좌표 검증 완료: x={x:.2f}m, y={y:.2f}m (odom 프레임)"
        )
        
        self.status = Status.SUCCESS
        return self.status


class NavigateToTarget(ActionWithROSAction):
    """
    Nav2를 사용한 목표 지점 이동
    
    기능:
    - blackboard의 target_position으로 이동
    - Nav2 액션 클라이언트 사용
    - odom 프레임 기준 좌표
    - 도착 확인 및 결과 반환
    
    Blackboard 입력:
    - 'target_position': (x, y, yaw) 목표 좌표 (odom 프레임)
    
    ROS2 Action:
    - /navigate_to_pose (nav2_msgs/action/NavigateToPose)
    """
    
    def __init__(self, name, agent):
        ns = agent.ros_namespace or ""
        action_name = f"{ns}/navigate_to_pose" if ns else "/navigate_to_pose"
        super().__init__(name, agent, (NavigateToPose, action_name))
        
        # 목표 위치 발행용 퍼블리셔
        goal_topic = f"{ns}/goal_pose" if ns else "/goal_pose"
        self.goal_pub = self.ros.node.create_publisher(PoseStamped, goal_topic, 10)
    
    def _build_goal(self, agent, bb):
        target = bb.get('target_position')
        if target is None:
            return None
        
        x, y = target[0], target[1]
        yaw = target[2] if len(target) > 2 else 0.0
        
        ps = PoseStamped()
        ps.header.frame_id = 'odom'  # ← odom 프레임 사용
        ps.header.stamp = self.ros.node.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = 0.0
        
        # yaw를 quaternion으로 변환
        ps.pose.orientation.z = math.sin(yaw * 0.5)
        ps.pose.orientation.w = math.cos(yaw * 0.5)
        
        self.ros.node.get_logger().info(
            f"🚗 Nav2 목표 전송: x={x:.2f}m, y={y:.2f}m (odom 프레임)"
        )
        
        goal = NavigateToPose.Goal()
        goal.pose = ps
        return goal
    
    def _on_running(self, agent, bb):
        """실행 중 목표 위치 재발행"""
        target = bb.get('target_position')
        if target is None:
            return
        
        x, y = target[0], target[1]
        yaw = target[2] if len(target) > 2 else 0.0
        
        ps = PoseStamped()
        ps.header.frame_id = 'odom'  # ← odom 프레임 사용
        ps.header.stamp = self.ros.node.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.z = math.sin(yaw * 0.5)
        ps.pose.orientation.w = math.cos(yaw * 0.5)
        
        self.goal_pub.publish(ps)
    
    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            bb['nav_result'] = 'succeeded'
            self.ros.node.get_logger().info("✅ 목표 지점 도착 완료!")
            return Status.SUCCESS
        elif status_code == GoalStatus.STATUS_CANCELED:
            bb['nav_result'] = 'canceled'
            self.ros.node.get_logger().warning("⚠️ 주행 취소됨")
            return Status.FAILURE
        else:
            bb['nav_result'] = 'aborted'
            self.ros.node.get_logger().error("❌ 주행 실패")
            return Status.FAILURE


class NavigateToWaypoint(ActionWithROSAction):
    """
    경로점(Waypoint) 이동
    
    기능:
    - blackboard의 current_waypoint로 이동
    - 경로점 순회에 사용
    - odom 프레임 기준
    
    Blackboard 입력:
    - 'current_waypoint': (x, y, yaw) 경로점 좌표 (odom 프레임)
    """
    
    def __init__(self, name, agent):
        ns = agent.ros_namespace or ""
        action_name = f"{ns}/navigate_to_pose" if ns else "/navigate_to_pose"
        super().__init__(name, agent, (NavigateToPose, action_name))
    
    def _build_goal(self, agent, bb):
        waypoint = bb.get('current_waypoint')
        if waypoint is None:
            return None
        
        x, y = waypoint[0], waypoint[1]
        yaw = waypoint[2] if len(waypoint) > 2 else 0.0
        
        ps = PoseStamped()
        ps.header.frame_id = 'odom'  # ← odom 프레임 사용
        ps.header.stamp = self.ros.node.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.z = math.sin(yaw * 0.5)
        ps.pose.orientation.w = math.cos(yaw * 0.5)
        
        goal = NavigateToPose.Goal()
        goal.pose = ps
        return goal
    
    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            self.ros.node.get_logger().info("✅ 경로점 도착")
            return Status.SUCCESS
        else:
            self.ros.node.get_logger().warning("⚠️ 경로점 주행 실패")
            return Status.FAILURE


class RotateToAngle(Node):
    """
    특정 각도로 회전
    
    기능:
    - 로봇을 지정된 각도로 회전
    - /cmd_vel 토픽으로 회전 속도 발행
    - 목표 각도 도달 시 정지
    
    주의:
    - 현재는 시간 기반 회전 (간단한 구현)
    - 실제 환경에서는 Odometry 피드백 필요
    
    Blackboard 입력:
    - 'target_angle': 목표 각도 (라디안)
    
    ROS2 Topic:
    - /cmd_vel (geometry_msgs/Twist) - 발행
    """
    
    def __init__(self, name, agent, target_angle=0.0, angular_speed=0.5):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.target_angle = target_angle
        self.angular_speed = angular_speed
        self.type = "Action"
        
        # cmd_vel 퍼블리셔
        ns = agent.ros_namespace or ""
        cmd_topic = f"{ns}/cmd_vel" if ns else "/cmd_vel"
        self.cmd_pub = self.ros.node.create_publisher(Twist, cmd_topic, 10)
        
        self.start_time = None
        self.rotating = False
    
    async def run(self, agent, blackboard):
        # blackboard에서 목표 각도 가져오기 (옵션)
        target = blackboard.get('target_angle', self.target_angle)
        
        if not self.rotating:
            # 회전 시작
            self.start_time = time.time()
            self.rotating = True
            
            self.ros.node.get_logger().info(
                f"🔄 회전 시작: {math.degrees(target):.1f}도"
            )
        
        # 회전 시간 계산 (간단한 추정)
        rotation_time = abs(target) / self.angular_speed
        elapsed = time.time() - self.start_time
        
        if elapsed < rotation_time:
            # 회전 명령 발행
            twist = Twist()
            twist.angular.z = self.angular_speed if target > 0 else -self.angular_speed
            self.cmd_pub.publish(twist)
            
            self.status = Status.RUNNING
            return self.status
        else:
            # 회전 완료 - 정지
            twist = Twist()
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            
            self.rotating = False
            self.ros.node.get_logger().info("✅ 회전 완료")
            
            self.status = Status.SUCCESS
            return self.status


class Wait(Node):
    """
    지정 시간 대기
    
    기능:
    - 지정된 시간(초) 동안 대기
    - 타이머 기반 비차단 대기
    
    사용 예시:
    <Wait duration="5.0"/>  <!-- 5초 대기 -->
    """
    
    def __init__(self, name, agent, duration=1.0):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.duration = duration
        self.start_time = None
        self.type = "Action"
    
    async def run(self, agent, blackboard):
        # blackboard에서 duration 가져오기 (옵션)
        duration = blackboard.get('wait_duration', self.duration)
        
        if self.start_time is None:
            # 대기 시작
            self.start_time = time.time()
            self.ros.node.get_logger().info(f"⏳ {duration}초 대기 시작")
        
        elapsed = time.time() - self.start_time
        
        if elapsed >= duration:
            # 대기 완료
            self.start_time = None
            self.ros.node.get_logger().info("✅ 대기 완료")
            self.status = Status.SUCCESS
            return self.status
        
        # 대기 중
        self.status = Status.RUNNING
        return self.status


class ConfirmVictimLocation(Node):
    """
    조난자 위치 재확인
    
    기능:
    - YOLO 탐지 결과와 현재 위치를 비교
    - 오차 범위 내이면 위치 확정
    - 재확인된 위치를 blackboard에 저장
    
    Blackboard 입력:
    - 'victim_detection': YOLO 탐지 결과
    - 'target_position': 예상 조난자 위치
    
    Blackboard 출력:
    - 'victim_location': 확정된 조난자 위치 (x, y)
    """
    
    def __init__(self, name, agent, tolerance=0.5):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.tolerance = tolerance  # 허용 오차 (미터)
        self.type = "Action"
    
    async def run(self, agent, blackboard):
        # 예상 위치
        target_pos = blackboard.get('target_position')
        
        # YOLO 탐지 결과
        victim_detected = blackboard.get('victim_detected', False)
        
        if not victim_detected:
            self.ros.node.get_logger().warn("⚠️ 조난자 미탐지")
            self.status = Status.FAILURE
            return self.status
        
        if target_pos is None:
            self.ros.node.get_logger().error("❌ 목표 위치 없음")
            self.status = Status.FAILURE
            return self.status
        
        # 위치 확정 (실제로는 YOLO 위치와 비교해야 함)
        # 여기서는 간단히 target_pos를 victim_location으로 저장
        victim_location = (target_pos[0], target_pos[1])
        blackboard['victim_location'] = victim_location
        blackboard['start_position'] = target_pos  # 경로 생성을 위한 시작점
        
        self.ros.node.get_logger().info(
            f"📍 조난자 위치 확정: ({victim_location[0]:.2f}, {victim_location[1]:.2f})"
        )
        
        self.status = Status.SUCCESS
        return self.status


class StartClueMonitoring(Node):
    """
    증거(단서) 감지 활성화
    
    기능:
    - /clue_detection 토픽 구독 시작
    - 감지된 단서를 blackboard에 저장
    - 지속적인 모니터링 (백그라운드)
    
    ROS2 Topic:
    - /clue_detection (std_msgs/String) - 구독
    
    Blackboard 출력:
    - 'clue_monitoring_active': True
    - 'detected_clues': 감지된 단서 리스트
    """
    
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.type = "Action"
        self.monitoring_active = False
        self.detected_clues = []
        
        # 단서 감지 토픽 구독
        self.clue_sub = self.ros.node.create_subscription(
            String,
            '/clue_detection',
            self._clue_callback,
            10
        )
    
    def _clue_callback(self, msg):
        """단서 감지 콜백"""
        if self.monitoring_active:
            clue_data = msg.data
            if clue_data not in self.detected_clues:
                self.detected_clues.append(clue_data)
                self.ros.node.get_logger().info(f"🔍 단서 발견: {clue_data}")
    
    async def run(self, agent, blackboard):
        if not self.monitoring_active:
            # 모니터링 시작
            self.monitoring_active = True
            blackboard['clue_monitoring_active'] = True
            blackboard['detected_clues'] = self.detected_clues
            
            self.ros.node.get_logger().info("👁️ 단서 모니터링 시작")
        
        # blackboard 업데이트
        blackboard['detected_clues'] = self.detected_clues
        
        self.status = Status.SUCCESS
        return self.status


class GetNextClue(Node):
    """
    다음 단서 가져오기
    
    기능:
    - blackboard의 detected_clues에서 다음 단서 추출
    - 단서가 없으면 FAILURE 반환
    - 단서를 current_clue에 저장
    
    Blackboard 입력:
    - 'detected_clues': 단서 리스트
    
    Blackboard 출력:
    - 'current_clue': 현재 처리할 단서
    """
    
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.type = "Action"
    
    async def run(self, agent, blackboard):
        clues = blackboard.get('detected_clues', [])
        
        if not clues or len(clues) == 0:
            self.ros.node.get_logger().info("ℹ️ 감지된 단서 없음")
            self.status = Status.FAILURE
            return self.status
        
        # 첫 번째 단서 가져오기
        current_clue = clues.pop(0)
        blackboard['current_clue'] = current_clue
        blackboard['detected_clues'] = clues  # 업데이트된 리스트 저장
        
        self.ros.node.get_logger().info(
            f"📋 단서 추출: {current_clue} (남은 단서: {len(clues)}개)"
        )
        
        self.status = Status.SUCCESS
        return self.status
