import math
import numpy as np
from modules.base_bt_nodes import BTNodeList, Status, Node, Sequence, Fallback, ReactiveSequence, ReactiveFallback

# BT Node List
CUSTOM_ACTION_NODES = [
    'MoveToTarget',
    'MoveToPosition',
    'RotateInPlace',
    'WaitForDuration',
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


# Import advanced action nodes
from .advanced_action_nodes import (
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
