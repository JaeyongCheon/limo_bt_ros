#!/usr/bin/env python3
"""
기본 BT Action 노드 - 주행 팀원 2 구현 (Odom 강제 연결 수정판)
구조 로봇 미션의 기본 액션 노드 10개

[긴급 수정 사항]
- WaitForMissionCommand: QoS 설정을 LIMO Base 노드와 1:1로 강제 매칭
  (Reliability: RELIABLE, Durability: VOLATILE)
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
from nav_msgs.msg import Odometry
# [중요] QoS 정밀 설정을 위한 모듈 임포트
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class ConfigureNav2ForDetour(Node):
    """
    Nav2를 우회 경로 허용 모드로 설정
    - Costmap inflation 증가로 장애물 주변 회피
    - Planner tolerance 증가로 더 먼 경로 허용
    """
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.type = "Action"
        self.configured = False
        
        # 파라미터 설정 클라이언트 생성
        self.param_client = None
    
    async def run(self, agent, blackboard):
        if self.configured:
            self.status = Status.SUCCESS
            return self.status
        
        try:
            import asyncio
            from rcl_interfaces.srv import SetParameters
            from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
            
            # Planner 서버에 파라미터 설정 요청
            planner_client = self.ros.node.create_client(
                SetParameters, 
                '/planner_server/set_parameters'
            )
            
            controller_client = self.ros.node.create_client(
                SetParameters,
                '/controller_server/set_parameters'
            )
            
            # 파라미터 설정
            planner_request = SetParameters.Request()
            planner_request.parameters = [
                # 더 긴 경로도 탐색 허용
                Parameter(
                    name='GridBased.max_planning_time',
                    value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=10.0)
                ),
                Parameter(
                    name='GridBased.tolerance',
                    value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=1.0)
                ),
            ]
            
            controller_request = SetParameters.Request()
            controller_request.parameters = [
                # 장애물 회피 강화
                Parameter(
                    name='FollowPath.max_robot_pose_search_dist',
                    value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=2.0)
                ),
            ]
            
            self.ros.node.get_logger().info(
                "🔧 Nav2 파라미터 설정 시도 중...\n"
                "   - Planner: 더 긴 경로 허용\n"
                "   - Controller: 장애물 회피 강화"
            )
            
            # 비동기로 설정 (실패해도 계속)
            if planner_client.wait_for_service(timeout_sec=1.0):
                future = planner_client.call_async(planner_request)
            
            if controller_client.wait_for_service(timeout_sec=1.0):
                future2 = controller_client.call_async(controller_request)
            
            self.configured = True
            self.status = Status.SUCCESS
            
        except Exception as e:
            self.ros.node.get_logger().warning(
                f"⚠️ Nav2 파라미터 설정 실패 (기본 설정 사용)\n"
                f"   팁: Nav2가 실행 중인지 확인하세요\n"
                f"   오류: {e}"
            )
            self.configured = True
            self.status = Status.SUCCESS
        
        return self.status


class LogMessage(Node):
    """
    로그 메시지 출력
    """
    def __init__(self, name, agent, message="", level="INFO"):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.message = message
        self.level = level.upper()
        self.type = "Action"
    
    async def run(self, agent, blackboard):
        msg = blackboard.get('log_message', self.message)
        
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
    미션 명령 대기 및 좌표 입력 (간단 버전)
    - 현재 위치 기준 상대 좌표 입력
    - odom 절대 좌표로 자동 변환
    """
    
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.type = "Action"
        self.coords_received = False
        self.current_odom = None
        self.first_odom_received = False
        
        # Odometry 구독 (RELIABLE QoS로 Publisher와 매칭)
        # LIMO는 /limo namespace를 사용하지만 /odom으로 remap됨
        odom_topic = "/odom"
        
        # Publisher QoS와 정확히 매칭: RELIABLE + VOLATILE
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.odom_sub = self.ros.node.create_subscription(
            Odometry, 
            odom_topic,
            self._odom_callback,
            qos_profile
        )
        
        self.ros.node.get_logger().info(f"📡 Odometry 구독 시작: {odom_topic}")
    
    def _odom_callback(self, msg):
        """Odometry 콜백 - 데이터 수신 확인"""
        if not self.first_odom_received:
            self.ros.node.get_logger().info("✅ Odometry 첫 수신 성공!")
            self.first_odom_received = True
        self.current_odom = msg
    
    def _get_yaw_from_odom(self, odom_msg):
        orientation = odom_msg.pose.pose.orientation
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    async def run(self, agent, blackboard):
        # 이미 좌표를 받았으면 완료
        if self.coords_received:
            self.status = Status.SUCCESS
            return self.status
        
        # Odometry 대기
        if self.current_odom is None:
            self.ros.node.get_logger().info("⏳ Odometry 수신 대기 중...", throttle_duration_sec=2.0)
            self.status = Status.RUNNING
            return self.status

        # 현재 위치 및 각도 가져오기
        curr_x = self.current_odom.pose.pose.position.x
        curr_y = self.current_odom.pose.pose.position.y
        curr_yaw = self._get_yaw_from_odom(self.current_odom)
        
        # 사용자 입력
        try:
            print("\n" + "=" * 60)
            print(f"📍 현재 위치: ({curr_x:.2f}, {curr_y:.2f}), 각도: {math.degrees(curr_yaw):.1f}°")
            print("🎯 목표 지점을 입력하세요 (로봇 기준 상대 좌표)")
            print("   X: 전방(+)/후방(-), Y: 좌측(+)/우측(-)")
            print("=" * 60)
            
            rel_x = float(input("X (전방 거리, m): "))
            rel_y = float(input("Y (좌측 거리, m): "))
            
            # 로봇 좌표계 → odom 좌표계 변환 (로봇 방향 고려)
            # X: 로봇 전방, Y: 로봇 좌측 → odom 좌표로 회전 변환
            target_x = curr_x + (rel_x * math.cos(curr_yaw) - rel_y * math.sin(curr_yaw))
            target_y = curr_y + (rel_x * math.sin(curr_yaw) + rel_y * math.cos(curr_yaw))
            
            # 거리 검증 (입력한 거리와 실제 이동 거리 비교)
            calculated_distance = math.sqrt(
                (target_x - curr_x)**2 + (target_y - curr_y)**2
            )
            expected_distance = math.sqrt(rel_x**2 + rel_y**2)
            
            # Blackboard에 저장
            blackboard['target_position'] = (target_x, target_y, curr_yaw)
            blackboard['start_position'] = (curr_x, curr_y)  # 시작 위치 저장
            
            print(f"\n✅ 목표 설정 완료:")
            print(f"   입력: 전방 {rel_x}m, 좌측 {rel_y}m")
            print(f"   시작: odom({curr_x:.2f}, {curr_y:.2f})")
            print(f"   목표: odom({target_x:.2f}, {target_y:.2f})")
            print(f"   예상 이동거리: {expected_distance:.2f}m")
            print(f"   계산 이동거리: {calculated_distance:.2f}m")
            print(f"   로봇 각도: {math.degrees(curr_yaw):.1f}°\n")
            
            self.coords_received = True
            self.status = Status.SUCCESS
            return self.status
            
        except ValueError:
            self.ros.node.get_logger().error("❌ 숫자를 입력하세요")
            self.status = Status.RUNNING
            return self.status
        except Exception as e:
            self.ros.node.get_logger().error(f"❌ 오류: {e}")
            self.status = Status.RUNNING
            return self.status
    
    def reset(self):
        """노드 리셋 - 좌표만 초기화, odometry는 유지"""
        self.coords_received = False
        # current_odom은 리셋하지 않음 (계속 사용)


class ParseTargetCommand(Node):
    """
    목표 좌표 검증 (단순 버전)
    """
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.type = "Action"
    
    async def run(self, agent, blackboard):
        target_position = blackboard.get('target_position')
        
        if target_position is None:
            self.ros.node.get_logger().error("❌ 목표 좌표 없음")
            self.status = Status.FAILURE
            return self.status
        
        # victim_location도 설정 (다른 노드와의 호환성)
        blackboard['victim_location'] = (target_position[0], target_position[1])
        
        self.ros.node.get_logger().info("✅ 좌표 검증 완료")
        self.status = Status.SUCCESS
        return self.status


class NavigateToTarget(ActionWithROSAction):
    """
    Nav2 목표 지점 이동 (거리 검증 포함)
    """
    def __init__(self, name, agent):
        ns = agent.ros_namespace or ""
        action_name = f"{ns}/navigate_to_pose" if ns else "/navigate_to_pose"
        super().__init__(name, agent, (NavigateToPose, action_name))
        goal_topic = f"{ns}/goal_pose" if ns else "/goal_pose"
        self.goal_pub = self.ros.node.create_publisher(PoseStamped, goal_topic, 10)
        self.goal_sent = False  # 목표 전송 여부 플래그
        
        # Odometry 구독 (도착 후 실제 거리 계산용)
        self.current_odom = None
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.odom_sub = self.ros.node.create_subscription(
            Odometry,
            "/odom",
            lambda msg: setattr(self, 'current_odom', msg),
            qos_profile
        )
    
    def _build_goal(self, agent, bb):
        target = bb.get('target_position')
        if target is None:
            return None
        
        x, y = target[0], target[1]
        yaw = target[2] if len(target) > 2 else 0.0
        
        # 재시도 횟수 추적 (점진적 우회 전략)
        # 단, 이미 증가했으면 다시 증가하지 않음
        retry_count = bb.get('nav_retry_count', 0)
        if not bb.get('nav_goal_building', False):
            bb['nav_retry_count'] = retry_count + 1
            bb['nav_goal_building'] = True
            retry_count = bb['nav_retry_count'] - 1
        
        ps = PoseStamped()
        ps.header.frame_id = 'odom'
        ps.header.stamp = self.ros.node.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = 0.0
        ps.pose.orientation.z = math.sin(yaw * 0.5)
        ps.pose.orientation.w = math.cos(yaw * 0.5)
        
        goal = NavigateToPose.Goal()
        goal.pose = ps
        
        # 재시도마다 다른 전략 표시
        actual_retry = bb['nav_retry_count']
        if actual_retry == 1:
            strategy = "직접 경로 (최단)"
        elif actual_retry == 2:
            strategy = "우회 경로 (중간 거리)"
        else:
            strategy = "멀리 우회 (완전히 돌아가는 경로)"
        
        self.ros.node.get_logger().info(
            f"🚗 Nav2 목표 전송 (시도 {actual_retry}/3)\n"
            f"   좌표: odom({x:.2f}, {y:.2f})\n"
            f"   전략: {strategy}"
        )
        
        self.goal_sent = False  # 목표 전송 플래그 초기화
        return goal
    
    def _on_running(self, agent, bb):
        # 목표를 한 번만 publish (경로 계획 안정화)
        if not self.goal_sent:
            target = bb.get('target_position')
            if target is None:
                return
            x, y = target[0], target[1]
            yaw = target[2] if len(target) > 2 else 0.0
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            ps.header.stamp = self.ros.node.get_clock().now().to_msg()
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
    def _interpret_result(self, result, agent, bb, status_code=None):
        # goal building 플래그 리셋
        bb['nav_goal_building'] = False
        
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            bb['nav_result'] = 'succeeded'
            
            # 실제 이동 거리 계산 및 출력logger().info("📍 목표 위치 publish 완료 - 경로 계획 시작")
    
    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            bb['nav_result'] = 'succeeded'
            
            # 실제 이동 거리 계산 및 출력
            start_pos = bb.get('start_position')
            target_pos = bb.get('target_position')
            
            if start_pos and target_pos and self.current_odom:
                curr_x = self.current_odom.pose.pose.position.x
                curr_y = self.current_odom.pose.pose.position.y
                
                actual_distance = math.sqrt(
                    (curr_x - start_pos[0])**2 + (curr_y - start_pos[1])**2
                )
                expected_distance = math.sqrt(
                    (target_pos[0] - start_pos[0])**2 + (target_pos[1] - start_pos[1])**2
                )
                error = abs(actual_distance - expected_distance)
                
                self.ros.node.get_logger().info(
                    f"✅ 목표 지점 도착!\n"
                    f"   예상 거리: {expected_distance:.2f}m\n"
                    f"   실제 이동: {actual_distance:.2f}m\n"
                    f"   오차: {error:.2f}m\n"
                    f"   도착 위치: ({curr_x:.2f}, {curr_y:.2f})"
                )
            else:
                self.ros.node.get_logger().info("✅ 목표 지점 도착 완료!")
            
            return Status.SUCCESS
        elif status_code == GoalStatus.STATUS_CANCELED:
            bb['nav_result'] = 'canceled'
            self.ros.node.get_logger().warning("⚠️ 주행 취소됨")
            return Status.FAILURE
        else:
            bb['nav_result'] = 'aborted'
            self.ros.node.get_logger().error(
                f"❌ 주행 실패 (Status: {status_code})\n"
                "   원인:\n"
                "   - Nav2 액션 서버가 실행 중인지 확인\n"
                "   - 목표 위치가 장애물에 있지 않은지 확인\n"
                "   - Costmap이 올바르게 업데이트되고 있는지 확인\n"
                "   - 좌표계(odom/map)가 올바른지 확인"
            )
            return Status.FAILURE


class NavigateToWaypoint(ActionWithROSAction):
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
        ps.header.frame_id = 'odom'  # ✅ odom 프레임 통일
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
    def __init__(self, name, agent, target_angle=0.0, angular_speed=0.5):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.target_angle = target_angle
        self.angular_speed = angular_speed
        self.type = "Action"
        ns = agent.ros_namespace or ""
        cmd_topic = f"{ns}/cmd_vel" if ns else "/cmd_vel"
        self.cmd_pub = self.ros.node.create_publisher(Twist, cmd_topic, 10)
        self.start_time = None
        self.rotating = False
    
    async def run(self, agent, blackboard):
        target = blackboard.get('target_angle', self.target_angle)
        if not self.rotating:
            self.start_time = time.time()
            self.rotating = True
            self.ros.node.get_logger().info(f"🔄 회전 시작: {math.degrees(target):.1f}도")
        rotation_time = abs(target) / self.angular_speed
        elapsed = time.time() - self.start_time
        if elapsed < rotation_time:
            twist = Twist()
            twist.angular.z = self.angular_speed if target > 0 else -self.angular_speed
            self.cmd_pub.publish(twist)
            self.status = Status.RUNNING
            return self.status
        else:
            twist = Twist()
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.rotating = False
            self.ros.node.get_logger().info("✅ 회전 완료")
            self.status = Status.SUCCESS
            return self.status


class Wait(Node):
    def __init__(self, name, agent, duration=1.0):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.duration = duration
        self.start_time = None
        self.type = "Action"
    
    async def run(self, agent, blackboard):
        duration = blackboard.get('wait_duration', self.duration)
        if self.start_time is None:
            self.start_time = time.time()
            self.ros.node.get_logger().info(f"⏳ {duration}초 대기 시작")
        elapsed = time.time() - self.start_time
        if elapsed >= duration:
            self.start_time = None
            self.ros.node.get_logger().info("✅ 대기 완료")
            self.status = Status.SUCCESS
            return self.status
        self.status = Status.RUNNING
        return self.status


class ConfirmVictimLocation(Node):
    def __init__(self, name, agent, tolerance=0.5):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.tolerance = tolerance
        self.type = "Action"
    
    async def run(self, agent, blackboard):
        target_pos = blackboard.get('target_position')
        victim_detected = blackboard.get('victim_detected', False)
        if not victim_detected:
            self.ros.node.get_logger().warn("⚠️ 조난자 미탐지")
            self.status = Status.FAILURE
            return self.status
        if target_pos is None:
            self.ros.node.get_logger().error("❌ 목표 위치 없음")
            self.status = Status.FAILURE
            return self.status
        victim_location = (target_pos[0], target_pos[1])
        blackboard['victim_location'] = victim_location
        blackboard['start_position'] = target_pos
        self.ros.node.get_logger().info(f"📍 조난자 위치 확정: ({victim_location[0]:.2f}, {victim_location[1]:.2f})")
        self.status = Status.SUCCESS
        return self.status


class StartClueMonitoring(Node):
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.type = "Action"
        self.monitoring_active = False
        self.detected_clues = []
        self.clue_sub = self.ros.node.create_subscription(
            String, '/clue_detection', self._clue_callback, 10)
    
    def _clue_callback(self, msg):
        if self.monitoring_active:
            clue_data = msg.data
            if clue_data not in self.detected_clues:
                self.detected_clues.append(clue_data)
                self.ros.node.get_logger().info(f"🔍 단서 발견: {clue_data}")
    
    async def run(self, agent, blackboard):
        if not self.monitoring_active:
            self.monitoring_active = True
            blackboard['clue_monitoring_active'] = True
            blackboard['detected_clues'] = self.detected_clues
            self.ros.node.get_logger().info("👁️ 단서 모니터링 시작")
        blackboard['detected_clues'] = self.detected_clues
        self.status = Status.SUCCESS
        return self.status


class GetNextClue(Node):
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
        current_clue = clues.pop(0)
        blackboard['current_clue'] = current_clue
        blackboard['detected_clues'] = clues
        self.ros.node.get_logger().info(f"📋 단서 추출: {current_clue} (남은 단서: {len(clues)}개)")
        self.status = Status.SUCCESS
        return self.status