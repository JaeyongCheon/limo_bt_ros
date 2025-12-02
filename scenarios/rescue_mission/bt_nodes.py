import math
import numpy as np
from modules.base_bt_nodes import BTNodeList, Status, Node, Sequence, Fallback, ReactiveSequence, ReactiveFallback

# BT Node List
CUSTOM_ACTION_NODES = [
    'MoveToTarget',
    'MoveToPosition',
    'RotateInPlace',
    'WaitForDuration',
    'UpdateOdometry',
    'GetUserInput',
    'NavigateToGoal',
    # Advanced Action Nodes
    'GenerateSpiralWaypoints',
    'GetNextWaypoint',
    'GenerateRescuePaths',
    'VisualizeResults',
    'PublishMissionSuccess',
    'ReturnToBase',
    'WaitForRescueTeam',
    'EscortToVictim',
    'CheckTeamFollowing',
    'WarnDangerZone',
    'AnnounceArrival',
]

CUSTOM_CONDITION_NODES = [
    'IsNearbyTarget',
    'IsAtPosition',
    'IsTerrainTraversable',
    'IsPathSafe',
    'HasGridMapData',
    'IsVictimDetected',
]

# BT Node List
BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)

# ROS2 imports
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from grid_map_msgs.msg import GridMap
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from modules.base_bt_nodes_ros import ConditionWithROSTopics, ActionWithROSAction


class IsNearbyTarget(ConditionWithROSTopics):
    """목표 위치에 근접했는지 확인 (Odometry 기반)"""
    def __init__(self, name, agent, default_thresh=0.5):
        ns = agent.ros_namespace or ""
        super().__init__(name, agent, [
            (Odometry, f"{ns}/odom" if ns else "/odom", 'odom'),
        ])
        self.default_thresh = default_thresh

    def _predicate(self, agent, blackboard):
        if "odom" not in self._cache:
            return False
        
        # 현재 로봇 위치
        odom = self._cache["odom"]
        robot_x = odom.pose.pose.position.x
        robot_y = odom.pose.pose.position.y
        
        # 목표 위치 (blackboard에서 가져옴)
        target = blackboard.get('target_position')
        if target is None:
            return False

        target_x, target_y = target[0], target[1]

        # 거리 계산
        thresh = blackboard.get("nearby_threshold", self.default_thresh)
        dist = math.hypot(robot_x - target_x, robot_y - target_y)
        blackboard["distance_to_target"] = dist
        
        return dist <= float(thresh)

class MoveToTarget(ActionWithROSAction):
    """Nav2를 사용하여 목표 위치로 이동"""
    def __init__(self, name, agent):
        ns = agent.ros_namespace or ""
        action_name = f"{ns}/navigate_to_pose" if ns else "/navigate_to_pose"
        super().__init__(name, agent, (NavigateToPose, action_name))
        
        # 목표 위치 발행용 퍼블리셔
        goal_topic = f"{ns}/goal_pose" if ns else "/goal_pose"
        self.goal_pub = self.ros.node.create_publisher(PoseStamped, goal_topic, 10)

    def _get_target_position(self, bb):
        """blackboard에서 목표 위치 추출"""
        target = bb.get('target_position')
        if target is None:
            return None
        
        # (x, y) 또는 (x, y, yaw) 형태 지원
        if isinstance(target, (list, tuple)) and len(target) >= 2:
            return target
        return None

    def _build_goal(self, agent, bb):
        target = self._get_target_position(bb)
        if target is None:
            return None
        
        x, y = target[0], target[1]
        yaw = target[2] if len(target) > 2 else 0.0

        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.ros.node.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = 0.0
        
        # yaw를 quaternion으로 변환
        ps.pose.orientation.z = math.sin(yaw * 0.5)
        ps.pose.orientation.w = math.cos(yaw * 0.5)

        goal = NavigateToPose.Goal()
        goal.pose = ps
        return goal

    def _on_running(self, agent, bb):
        """RUNNING 중 목표 위치 발행"""
        target = self._get_target_position(bb)
        if target is None:
            return
        
        x, y = target[0], target[1]
        yaw = target[2] if len(target) > 2 else 0.0
        
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.ros.node.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.z = math.sin(yaw * 0.5)
        ps.pose.orientation.w = math.cos(yaw * 0.5)
        
        self.goal_pub.publish(ps)

    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            bb['nav_result'] = 'succeeded'
            return Status.SUCCESS
        elif status_code == GoalStatus.STATUS_CANCELED:
            bb['nav_result'] = 'canceled'
            return Status.FAILURE
        else:
            bb['nav_result'] = 'aborted'
            return Status.FAILURE


class IsAtPosition(ConditionWithROSTopics):
    """특정 위치에 도달했는지 확인"""
    def __init__(self, name, agent, position=None, threshold=0.3):
        ns = agent.ros_namespace or ""
        super().__init__(name, agent, [
            (Odometry, f"{ns}/odom" if ns else "/odom", 'odom'),
        ])
        self.position = position  # (x, y)
        self.threshold = threshold
    
    def _predicate(self, agent, blackboard):
        if "odom" not in self._cache:
            return False
        
        odom = self._cache["odom"]
        robot_x = odom.pose.pose.position.x
        robot_y = odom.pose.pose.position.y
        
        # blackboard 또는 초기화 시 설정된 position 사용
        pos = blackboard.get('check_position', self.position)
        if pos is None:
            return False
        
        target_x, target_y = pos[0], pos[1]
        dist = math.hypot(robot_x - target_x, robot_y - target_y)
        
        return dist <= self.threshold


class HasGridMapData(ConditionWithROSTopics):
    """GridMap 데이터가 수신되었는지 확인"""
    def __init__(self, name, agent):
        super().__init__(name, agent, [
            (GridMap, '/grid_map', 'grid_map'),
        ])
    
    def _predicate(self, agent, blackboard):
        if 'grid_map' not in self._cache:
            return False
        
        grid_map = self._cache['grid_map']
        # GridMap을 blackboard에 저장
        blackboard['grid_map'] = grid_map
        
        # 필요한 레이어가 있는지 확인
        required_layers = ['traversability', 'elevation', 'slope', 'roughness']
        for layer in required_layers:
            if layer not in grid_map.layers:
                return False
        
        return True


class IsTerrainTraversable(ConditionWithROSTopics):
    """목표 위치의 지형이 통과 가능한지 확인"""
    def __init__(self, name, agent, min_traversability=0.5):
        super().__init__(name, agent, [
            (GridMap, '/grid_map', 'grid_map'),
            (Odometry, f"{agent.ros_namespace or ''}/odom" if agent.ros_namespace else "/odom", 'odom'),
        ])
        self.min_traversability = min_traversability
    
    def _predicate(self, agent, blackboard):
        if 'grid_map' not in self._cache:
            return False
        
        grid_map = self._cache['grid_map']
        target = blackboard.get('target_position')
        
        if target is None:
            return False
        
        # GridMap에서 목표 위치의 traversability 확인
        try:
            # traversability 레이어 찾기
            if 'traversability' not in grid_map.layers:
                return False
            
            layer_idx = grid_map.layers.index('traversability')
            layer_data = grid_map.data[layer_idx].data
            
            # GridMap 정보
            resolution = grid_map.info.resolution
            length_x = grid_map.info.length_x
            length_y = grid_map.info.length_y
            center_x = grid_map.info.pose.position.x
            center_y = grid_map.info.pose.position.y
            
            width = int(length_x / resolution)
            height = int(length_y / resolution)
            
            # 목표 위치를 그리드 인덱스로 변환
            target_x, target_y = target[0], target[1]
            
            # 상대 위치 계산
            rel_x = target_x - (center_x - length_x / 2.0)
            rel_y = target_y - (center_y - length_y / 2.0)
            
            grid_x = int(rel_x / resolution)
            grid_y = int(rel_y / resolution)
            
            # 범위 체크
            if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
                return False
            
            # traversability 값 확인
            idx = grid_y * width + grid_x
            if idx >= len(layer_data):
                return False
            
            traversability = layer_data[idx]
            
            # NaN 체크
            if math.isnan(traversability):
                return False
            
            blackboard['target_traversability'] = traversability
            
            return traversability >= self.min_traversability
            
        except Exception as e:
            # 에러 발생 시 안전하게 False 반환
            return False


class IsPathSafe(ConditionWithROSTopics):
    """현재 위치에서 목표까지의 경로가 안전한지 확인"""
    def __init__(self, name, agent, max_slope=0.5, max_roughness=0.2):
        ns = agent.ros_namespace or ""
        super().__init__(name, agent, [
            (GridMap, '/grid_map', 'grid_map'),
            (Odometry, f"{ns}/odom" if ns else "/odom", 'odom'),
        ])
        self.max_slope = max_slope
        self.max_roughness = max_roughness
    
    def _predicate(self, agent, blackboard):
        if 'grid_map' not in self._cache or 'odom' not in self._cache:
            return False
        
        grid_map = self._cache['grid_map']
        odom = self._cache['odom']
        target = blackboard.get('target_position')
        
        if target is None:
            return False
        
        # 현재 위치
        robot_x = odom.pose.pose.position.x
        robot_y = odom.pose.pose.position.y
        
        # 간단한 직선 경로 체크
        # 실제로는 더 정교한 경로 계획이 필요
        try:
            if 'slope' not in grid_map.layers or 'roughness' not in grid_map.layers:
                return True  # 데이터 없으면 일단 통과
            
            slope_idx = grid_map.layers.index('slope')
            roughness_idx = grid_map.layers.index('roughness')
            
            slope_data = grid_map.data[slope_idx].data
            roughness_data = grid_map.data[roughness_idx].data
            
            resolution = grid_map.info.resolution
            length_x = grid_map.info.length_x
            length_y = grid_map.info.length_y
            center_x = grid_map.info.pose.position.x
            center_y = grid_map.info.pose.position.y
            
            width = int(length_x / resolution)
            
            # 직선 경로상의 몇 개 지점만 샘플링
            num_samples = 10
            target_x, target_y = target[0], target[1]
            
            for i in range(num_samples):
                t = i / float(num_samples - 1) if num_samples > 1 else 0.0
                check_x = robot_x + t * (target_x - robot_x)
                check_y = robot_y + t * (target_y - robot_y)
                
                # 그리드 인덱스 변환
                rel_x = check_x - (center_x - length_x / 2.0)
                rel_y = check_y - (center_y - length_y / 2.0)
                
                grid_x = int(rel_x / resolution)
                grid_y = int(rel_y / resolution)
                
                if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= int(length_y / resolution):
                    continue
                
                idx = grid_y * width + grid_x
                
                if idx >= len(slope_data) or idx >= len(roughness_data):
                    continue
                
                slope = slope_data[idx]
                roughness = roughness_data[idx]
                
                # NaN 체크
                if math.isnan(slope) or math.isnan(roughness):
                    continue
                
                # 안전 기준 초과 시 False
                if slope > self.max_slope or roughness > self.max_roughness:
                    blackboard['path_unsafe_reason'] = f'slope={slope:.2f}, roughness={roughness:.2f}'
                    return False
            
            return True
            
        except Exception as e:
            return True  # 에러 시 안전하게 통과


class IsVictimDetected(ConditionWithROSTopics):
    """조난자 발견 여부 확인 (vision_msgs/Detection2DArray 기반)"""
    def __init__(self, name, agent):
        try:
            from vision_msgs.msg import Detection2DArray
            super().__init__(name, agent, [
                (Detection2DArray, '/victim_detection', 'detections'),
            ])
        except ImportError:
            # vision_msgs 없으면 std_msgs.Bool 사용
            from std_msgs.msg import Bool
            super().__init__(name, agent, [
                (Bool, '/victim_detection', 'detection_bool'),
            ])
        ns = agent.ros_namespace or ""
        # Odometry도 구독하여 조난자 위치 계산
        self.odom_sub = self.ros.node.create_subscription(
            Odometry, f"{ns}/odom" if ns else "/odom",
            lambda msg: setattr(self, '_odom', msg),
            10
        )
        self._odom = None
    
    def _predicate(self, agent, blackboard):
        # vision_msgs 사용
        if 'detections' in self._cache:
            detections = self._cache['detections']
            if len(detections.detections) > 0:
                # 조난자 발견!
                # 현재 위치에서 일정 거리 앞을 조난자 위치로 설정
                if self._odom is not None:
                    robot_x = self._odom.pose.pose.position.x
                    robot_y = self._odom.pose.pose.position.y
                    
                    # 로봇 방향 계산
                    q = self._odom.pose.pose.orientation
                    yaw = math.atan2(
                        2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                    )
                    
                    # 조난자는 로봇 전방 3m로 가정
                    victim_distance = 3.0
                    victim_x = robot_x + victim_distance * math.cos(yaw)
                    victim_y = robot_y + victim_distance * math.sin(yaw)
                    
                    blackboard['victim_location'] = (victim_x, victim_y)
                    blackboard['victim_detected'] = True
                    
                    self.ros.node.get_logger().info(
                        f"🆘 조난자 발견! 위치: ({victim_x:.2f}, {victim_y:.2f})"
                    )
                    return True
        
        # std_msgs.Bool 사용
        if 'detection_bool' in self._cache:
            detection = self._cache['detection_bool']
            if detection.data:
                # 조난자 발견 (위치는 별도 설정)
                if 'victim_location' not in blackboard:
                    # 기본 위치 설정 (현재 위치에서 3m 앞)
                    if self._odom is not None:
                        robot_x = self._odom.pose.pose.position.x
                        robot_y = self._odom.pose.pose.position.y
                        blackboard['victim_location'] = (robot_x + 3.0, robot_y)
                
                blackboard['victim_detected'] = True
                return True
        
        return False


class MoveToPosition(ActionWithROSAction):
    """특정 (x, y) 좌표로 이동"""
    def __init__(self, name, agent, position=None):
        ns = agent.ros_namespace or ""
        action_name = f"{ns}/navigate_to_pose" if ns else "/navigate_to_pose"
        super().__init__(name, agent, (NavigateToPose, action_name))
        self.position = position  # 초기화 시 설정 가능
    
    def _build_goal(self, agent, bb):
        # blackboard 또는 초기화 위치 사용
        pos = bb.get('move_to_position', self.position)
        if pos is None:
            return None
        
        x, y = pos[0], pos[1]
        yaw = pos[2] if len(pos) > 2 else 0.0
        
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.ros.node.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = 0.0
        ps.pose.orientation.z = math.sin(yaw * 0.5)
        ps.pose.orientation.w = math.cos(yaw * 0.5)
        
        goal = NavigateToPose.Goal()
        goal.pose = ps
        return goal
    
    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            return Status.SUCCESS
        else:
            return Status.FAILURE


class RotateInPlace(Node):
    """제자리에서 회전"""
    def __init__(self, name, agent, target_yaw=0.0, angular_speed=0.5):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.target_yaw = target_yaw
        self.angular_speed = angular_speed
        
        ns = agent.ros_namespace or ""
        cmd_topic = f"{ns}/cmd_vel" if ns else "/cmd_vel"
        self.cmd_pub = self.ros.node.create_publisher(Twist, cmd_topic, 10)
        
        # Odometry 구독
        odom_topic = f"{ns}/odom" if ns else "/odom"
        self.odom_sub = self.ros.node.create_subscription(
            Odometry, odom_topic,
            lambda msg: setattr(self, 'current_odom', msg),
            10
        )
        self.current_odom = None
        self.type = "Action"
    
    async def run(self, agent, blackboard):
        if self.current_odom is None:
            self.status = Status.RUNNING
            return self.status
        
        # 현재 yaw 계산
        q = self.current_odom.pose.pose.orientation
        current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        
        # 목표 yaw (blackboard 우선)
        target = blackboard.get('rotate_to_yaw', self.target_yaw)
        
        # 각도 차이
        yaw_diff = target - current_yaw
        # [-pi, pi]로 정규화
        while yaw_diff > math.pi:
            yaw_diff -= 2.0 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2.0 * math.pi
        
        # 목표 도달 확인
        if abs(yaw_diff) < 0.1:  # 약 5도
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            self.status = Status.SUCCESS
            return self.status
        
        # 회전 명령
        cmd = Twist()
        cmd.angular.z = self.angular_speed if yaw_diff > 0 else -self.angular_speed
        self.cmd_pub.publish(cmd)
        
        self.status = Status.RUNNING
        return self.status


class WaitForDuration(Node):
    """지정된 시간 동안 대기"""
    def __init__(self, name, agent, duration=1.0):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.duration = duration
        self.start_time = None
        self.type = "Action"
    
    async def run(self, agent, blackboard):
        # 시작 시간 기록
        if self.start_time is None:
            self.start_time = self.ros.node.get_clock().now()
            self.status = Status.RUNNING
            return self.status
        
        # 경과 시간 확인
        elapsed = (self.ros.node.get_clock().now() - self.start_time).nanoseconds / 1e9
        duration = blackboard.get('wait_duration', self.duration)

        if elapsed >= duration:
            self.start_time = None  # 리셋
            self.status = Status.SUCCESS
            return self.status
        
        self.status = Status.RUNNING
        return self.status


class UpdateOdometry(Node):
    """Odometry를 blackboard에 업데이트 (항상 SUCCESS)"""
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        ns = agent.ros_namespace or ""
        
        # Odometry 구독
        odom_topic = f"{ns}/odom" if ns else "/odom"
        self.odom_sub = self.ros.node.create_subscription(
            Odometry, odom_topic,
            lambda msg: setattr(self, '_odom', msg),
            10
        )
        self._odom = None
        self.type = "Action"
    
    async def run(self, agent, blackboard):
        if self._odom is not None:
            # blackboard에 odometry 저장
            blackboard['odom'] = self._odom
            
            # 현재 위치도 저장
            blackboard['current_position'] = (
                self._odom.pose.pose.position.x,
                self._odom.pose.pose.position.y
            )
            
            # start_position이 없으면 설정 (최초 1회)
            if 'start_position' not in blackboard:
                blackboard['start_position'] = blackboard['current_position']
            
            self.status = Status.SUCCESS
        else:
            # 시뮬레이션 모드: Odometry가 없으면 기본값 사용
            if 'current_position' not in blackboard:
                blackboard['current_position'] = (0.0, 0.0)
                blackboard['start_position'] = (0.0, 0.0)
                blackboard['current_yaw'] = 0.0
                print("[UpdateOdometry] No odom, using default (0,0)")
            self.status = Status.SUCCESS
        
        return self.status


class GetUserInput(Node):
    """사용자로부터 목표 좌표 입력받기 (맵 불필요)
    
    터미널에서 'x y' 형식으로 입력 (예: 3.0 2.5)
    
    Parameters:
    - timeout: 입력 대기 시간 (초, 기본값: 30.0)
    - relative: True이면 상대좌표로 해석 (기본값: False)
    """
    def __init__(self, name, agent, timeout=30.0, relative=False):
        super().__init__(name)
        self.timeout = float(timeout)
        self.relative = relative
        self.type = "Action"
        self.input_received = False
        self.goal = None
    
    async def run(self, agent, blackboard):
        if not self.input_received:
            print("\n" + "="*50)
            if self.relative:
                current = blackboard.get('current_position', (0.0, 0.0))
                print(f"[GetUserInput] Current position: ({current[0]:.2f}, {current[1]:.2f})")
                print(f"[GetUserInput] Enter RELATIVE goal (dx dy): ", end='', flush=True)
            else:
                print(f"[GetUserInput] Enter ABSOLUTE goal (x y): ", end='', flush=True)
            
            try:
                user_input = input()
                parts = user_input.strip().split()
                
                if len(parts) >= 2:
                    x = float(parts[0])
                    y = float(parts[1])
                    
                    if self.relative:
                        current_pos = blackboard.get('current_position', (0.0, 0.0))
                        goal_x = current_pos[0] + x
                        goal_y = current_pos[1] + y
                        print(f"[GetUserInput] Relative ({x}, {y}) -> Goal: ({goal_x:.2f}, {goal_y:.2f})")
                    else:
                        goal_x = x
                        goal_y = y
                        print(f"[GetUserInput] Absolute Goal: ({goal_x:.2f}, {goal_y:.2f})")
                    
                    blackboard['goal_position'] = (goal_x, goal_y)
                    self.input_received = True
                    self.status = Status.SUCCESS
                else:
                    print("[GetUserInput] Invalid input format. Use: x y")
                    self.status = Status.FAILURE
            except ValueError as e:
                print(f"[GetUserInput] Invalid number format: {e}")
                self.status = Status.FAILURE
            except Exception as e:
                print(f"[GetUserInput] Error: {e}")
                self.status = Status.FAILURE
        else:
            self.status = Status.SUCCESS
        
        print("="*50 + "\n")
        return self.status
    
    def halt(self):
        self.input_received = False


class NavigateToGoal(Node):
    """맵 없이 오도메트리만으로 목표 좌표까지 이동
    
    순수 오도메트리 기반으로 목표까지 이동 (Nav2 불필요)
    cmd_vel로 직접 제어
    
    Parameters:
    - speed: 이동 속도 (m/s, 기본값: 0.3)
    - angular_speed: 회전 속도 (rad/s, 기본값: 0.5)
    - goal_threshold: 목표 도달 판정 거리 (m, 기본값: 0.2)
    - angle_threshold: 방향 정렬 판정 각도 (degree, 기본값: 10)
    """
    def __init__(self, name, agent, speed=0.3, angular_speed=0.5, 
                 goal_threshold=0.2, angle_threshold=10.0):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.speed = float(speed)
        self.angular_speed = float(angular_speed)
        self.goal_threshold = float(goal_threshold)
        self.angle_threshold = float(angle_threshold) * math.pi / 180.0
        self.type = "Action"
        
        ns = agent.ros_namespace or ""
        cmd_topic = f"/{ns}/cmd_vel" if ns else "/cmd_vel"
        self.cmd_pub = self.ros.node.create_publisher(Twist, cmd_topic, 10)
        print(f"[NavigateToGoal] Publishing to: {cmd_topic}")
        
        # Odometry 구독 (방향 정보 필요)
        odom_topic = f"/{ns}/odom" if ns else "/odom"
        self.odom_sub = self.ros.node.create_subscription(
            Odometry, odom_topic,
            self._odom_callback,
            10
        )
        print(f"[NavigateToGoal] Subscribed to: {odom_topic}")
        self.current_yaw = 0.0
    
    def _odom_callback(self, msg):
        """Odometry에서 현재 방향(yaw) 추출"""
        q = msg.pose.pose.orientation
        # Quaternion to Euler (yaw)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    async def run(self, agent, blackboard):
        # 목표 위치 확인
        goal = blackboard.get('goal_position')
        if goal is None:
            print("[NavigateToGoal] No goal position in blackboard")
            self.status = Status.FAILURE
            return self.status
        
        # 현재 위치 확인
        current_pos = blackboard.get('current_position')
        if current_pos is None:
            print("[NavigateToGoal] No current position available")
            self.status = Status.FAILURE
            return self.status
        
        goal_x, goal_y = goal[0], goal[1]
        curr_x, curr_y = current_pos[0], current_pos[1]
        
        # 목표까지 거리 계산
        dx = goal_x - curr_x
        dy = goal_y - curr_y
        distance = math.hypot(dx, dy)
        
        print(f"[NavigateToGoal] Current: ({curr_x:.2f}, {curr_y:.2f}), Goal: ({goal_x:.2f}, {goal_y:.2f}), Distance: {distance:.2f}m")
        
        # 목표 도달 확인
        if distance < self.goal_threshold:
            twist = Twist()
            self.cmd_pub.publish(twist)
            print(f"[NavigateToGoal] Goal reached! Distance: {distance:.2f}m")
            self.status = Status.SUCCESS
            return self.status
        
        # 목표 방향 계산
        target_yaw = math.atan2(dy, dx)
        yaw_error = target_yaw - self.current_yaw
        
        # 각도를 -pi ~ pi 범위로 정규화
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi
        
        twist = Twist()
        
        # 방향이 크게 틀어져 있으면 제자리 회전
        if abs(yaw_error) > self.angle_threshold:
            twist.angular.z = self.angular_speed if yaw_error > 0 else -self.angular_speed
            print(f"[NavigateToGoal] Turning... yaw_error: {yaw_error*180/math.pi:.1f}°, angular.z: {twist.angular.z:.2f}")
        else:
            # 방향이 맞으면 전진
            twist.linear.x = min(self.speed, distance)  # 가까우면 속도 감소
            # 미세 조정
            twist.angular.z = 0.3 * yaw_error
            print(f"[NavigateToGoal] Moving... distance: {distance:.2f}m, linear.x: {twist.linear.x:.2f}, angular.z: {twist.angular.z:.2f}")
        
        print(f"[NavigateToGoal] Publishing Twist - linear.x: {twist.linear.x}, angular.z: {twist.angular.z}")
        self.cmd_pub.publish(twist)
        self.status = Status.RUNNING
        return self.status
    
    def halt(self):
        """정지"""
        twist = Twist()
        self.cmd_pub.publish(twist)
        print("[NavigateToGoal] Stopped")


# Import advanced action nodes
try:
    from scenarios.rescue_mission.advanced_action_nodes import (
        GenerateSpiralWaypoints,
        GetNextWaypoint,
        GenerateRescuePaths,
        VisualizeResults,
        PublishMissionSuccess,
        ReturnToBase,
        WaitForRescueTeam,
        EscortToVictim,
        CheckTeamFollowing,
        WarnDangerZone,
        AnnounceArrival,
    )
except ImportError:
    # For direct execution
    import sys
    import os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from advanced_action_nodes import (
        GenerateSpiralWaypoints,
        GetNextWaypoint,
        GenerateRescuePaths,
        VisualizeResults,
        PublishMissionSuccess,
        ReturnToBase,
        WaitForRescueTeam,
        EscortToVictim,
        CheckTeamFollowing,
        WarnDangerZone,
        AnnounceArrival,
    )
