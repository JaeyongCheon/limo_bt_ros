#!/usr/bin/env python3
# condition_nodes.py
# Behavior Tree 조건 노드 - 커스텀 BT 프레임워크 버전
#
# 노드: CheckSensorStatus, IsTerrainAnalysisReady, IsYOLOReady,
#       IsNav2Ready, IsVictimDetected, HasCluesStored, IsReturnModeEnabled

from modules.base_bt_nodes_ros import ConditionWithROSTopics
from std_msgs.msg import Bool, String
from grid_map_msgs.msg import GridMap
from vision_msgs.msg import Detection2DArray


class CheckSensorStatus(ConditionWithROSTopics):
    """
    센서 상태 점검
    blackboard.sensor_ok = True/False
    """
    def __init__(self, name, agent):
        super().__init__(name, agent, [
            (Bool, '/sensor_status', 'sensor_ok'),
        ])

    def _predicate(self, agent, blackboard):
        if 'sensor_ok' not in self._cache:
            # 토픽이 없으면 일단 True로 가정
            agent.ros_bridge.node.get_logger().info("✅ 센서 상태 정상 (토픽 없음, 기본값)")
            return True
        
        sensor_ok = self._cache['sensor_ok']
        blackboard['sensor_ok'] = sensor_ok.data if hasattr(sensor_ok, 'data') else True
        
        if blackboard['sensor_ok']:
            agent.ros_bridge.node.get_logger().info("✅ 센서 상태 정상")
        else:
            agent.ros_bridge.node.get_logger().warning("⚠️ 센서 이상 감지")
        
        return blackboard['sensor_ok']


class IsTerrainAnalysisReady(ConditionWithROSTopics):
    """
    grid_map 토픽 도착 여부
    blackboard.terrain_ready = True
    """
    def __init__(self, name, agent):
        super().__init__(name, agent, [
            (GridMap, '/grid_map', 'grid_map'),
        ])

    def _predicate(self, agent, blackboard):
        if 'grid_map' not in self._cache:
            return False
        
        grid_map = self._cache['grid_map']
        blackboard['terrain_ready'] = True
        blackboard['grid_map'] = grid_map
        
        # GridMap이 유효한 데이터를 가지고 있는지 확인
        return len(grid_map.layers) > 0


class IsYOLOReady(ConditionWithROSTopics):
    """
    YOLO 탐지 데이터 확인
    blackboard.yolo_ready = True
    """
    def __init__(self, name, agent):
        super().__init__(name, agent, [
            (Detection2DArray, '/yolo/detections', 'yolo_detections'),
        ])

    def _predicate(self, agent, blackboard):
        if 'yolo_detections' not in self._cache:
            return False
        
        detections = self._cache['yolo_detections']
        blackboard['yolo_ready'] = True
        blackboard['yolo_detections'] = detections
        
        # 토픽이 발행되고 있으면 준비된 것으로 간주
        return True


class IsNav2Ready(ConditionWithROSTopics):
    """
    Nav2 액션 서버 연결 여부
    blackboard.nav2_ready = True
    """
    def __init__(self, name, agent):
        super().__init__(name, agent, [
            (Bool, '/nav2_status', 'nav2_ready'),
        ])
        self.action_client_checked = False
        self.is_ready = False

    def _predicate(self, agent, blackboard):
        # 액션 클라이언트 가용성 체크 (한 번만)
        if not self.action_client_checked:
            # NavigateToPose 액션 서버 체크
            from nav2_msgs.action import NavigateToPose
            from rclpy.action import ActionClient
            
            ns = agent.ros_namespace or ""
            action_name = f"{ns}/navigate_to_pose" if ns else "/navigate_to_pose"
            
            # 임시 액션 클라이언트 생성하여 서버 확인
            temp_client = ActionClient(self.ros.node, NavigateToPose, action_name)
            self.is_ready = temp_client.wait_for_server(timeout_sec=0.1)
            self.action_client_checked = True
            
            if self.is_ready:
                blackboard['nav2_ready'] = True
        
        return self.is_ready


class IsVictimDetected(ConditionWithROSTopics):
    """
    조난자 탐지 여부
    blackboard.victim_detected = True or victim_info
    """
    def __init__(self, name, agent):
        super().__init__(name, agent, [
            (Bool, '/victim_detection', 'victim_detected'),
            (String, '/victim_info', 'victim_info'),
        ])

    def _predicate(self, agent, blackboard):
        if 'victim_detected' not in self._cache:
            return False
        
        victim_detected = self._cache['victim_detected']
        
        if hasattr(victim_detected, 'data') and victim_detected.data:
            blackboard['victim_detected'] = True
            
            # 조난자 정보가 있으면 저장
            if 'victim_info' in self._cache:
                victim_info = self._cache['victim_info']
                blackboard['victim_info'] = victim_info.data if hasattr(victim_info, 'data') else str(victim_info)
            
            agent.ros_bridge.node.get_logger().info("🎯 조난자 발견!")
            return True
        
        return False


class HasCluesStored(ConditionWithROSTopics):
    """
    단서 목록이 있는지 확인
    blackboard.clue_list = [] or list
    """
    def __init__(self, name, agent):
        super().__init__(name, agent, [
            (String, '/clue_updates', 'clue_update'),
        ])
        self.clue_list = []

    def _predicate(self, agent, blackboard):
        # 새로운 단서가 발행되면 리스트에 추가
        if 'clue_update' in self._cache:
            clue_update = self._cache['clue_update']
            if hasattr(clue_update, 'data'):
                clue_data = clue_update.data
                if clue_data and clue_data not in self.clue_list:
                    self.clue_list.append(clue_data)
        
        # blackboard에서 clue_list 확인
        if 'clue_list' not in blackboard:
            blackboard['clue_list'] = self.clue_list
        else:
            # blackboard의 리스트 업데이트
            existing = blackboard.get('clue_list', [])
            if isinstance(existing, list):
                self.clue_list = existing
        
        return len(self.clue_list) > 0


class IsReturnModeEnabled(ConditionWithROSTopics):
    """
    귀환 모드 활성화 여부
    blackboard.return_mode = True/False
    """
    def __init__(self, name, agent):
        super().__init__(name, agent, [
            (Bool, '/return_mode', 'return_mode'),
            (String, '/mission_state', 'mission_state'),
        ])

    def _predicate(self, agent, blackboard):
        # return_mode 토픽 확인
        if 'return_mode' in self._cache:
            return_mode = self._cache['return_mode']
            if hasattr(return_mode, 'data'):
                blackboard['return_mode'] = return_mode.data
                return return_mode.data
        
        # mission_state가 RETURNING이면 귀환 모드
        if 'mission_state' in self._cache:
            mission_state = self._cache['mission_state']
            if hasattr(mission_state, 'data'):
                state = mission_state.data
                if state == 'RETURNING':
                    blackboard['return_mode'] = True
                    return True
        
        # blackboard에서 확인
        return blackboard.get('return_mode', False)


