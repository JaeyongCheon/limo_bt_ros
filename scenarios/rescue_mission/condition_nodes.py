#!/usr/bin/env python3
# condition_nodes.py
# Behavior Tree 조건 노드 스켈레톤 (사용중인 BT 라이브러리 맞춰서 수정하세요)
#
# 노드: CheckSensorStatus, IsTerrainAnalysisReady, IsYOLOReady,
#       IsNav2Ready, IsVictimDetected, HasCluesStored, IsReturnModeEnabled

import py_trees


class CheckSensorStatus(py_trees.behaviour.Behaviour):
    """
    센서 상태 점검
    blackboard.sensor_ok = True/False
    """
    def __init__(self, name="CheckSensorStatus"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("sensor_ok", access=py_trees.common.Access.READ)

    def update(self):
        if getattr(self.blackboard, "sensor_ok", False):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class IsTerrainAnalysisReady(py_trees.behaviour.Behaviour):
    """
    grid_map 토픽 도착 여부
    blackboard.terrain_ready = True
    """
    def __init__(self, name="IsTerrainAnalysisReady"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("terrain_ready", access=py_trees.common.Access.READ)

    def update(self):
        if getattr(self.blackboard, "terrain_ready", False):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class IsYOLOReady(py_trees.behaviour.Behaviour):
    """
    YOLO 탐지 데이터 확인
    blackboard.yolo_ready = True
    """
    def __init__(self, name="IsYOLOReady"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("yolo_ready", access=py_trees.common.Access.READ)

    def update(self):
        if getattr(self.blackboard, "yolo_ready", False):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class IsNav2Ready(py_trees.behaviour.Behaviour):
    """
    Nav2 액션 서버 연결 여부
    blackboard.nav2_ready = True
    """
    def __init__(self, name="IsNav2Ready"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("nav2_ready", access=py_trees.common.Access.READ)

    def update(self):
        if getattr(self.blackboard, "nav2_ready", False):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class IsVictimDetected(py_trees.behaviour.Behaviour):
    """
    조난자 탐지 여부
    blackboard.victim_detected = True or victim_info
    """
    def __init__(self, name="IsVictimDetected"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("victim_detected", access=py_trees.common.Access.READ)

    def update(self):
        if getattr(self.blackboard, "victim_detected", False):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class HasCluesStored(py_trees.behaviour.Behaviour):
    """
    단서 목록이 있는지 확인
    blackboard.clue_list = [] or list
    """
    def __init__(self, name="HasCluesStored"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("clue_list", access=py_trees.common.Access.READ)

    def update(self):
        clues = getattr(self.blackboard, "clue_list", [])
        if clues and len(clues) > 0:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class IsReturnModeEnabled(py_trees.behaviour.Behaviour):
    """
    귀환 모드 활성화 여부
    blackboard.return_mode = True/False
    """
    def __init__(self, name="IsReturnModeEnabled"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("return_mode", access=py_trees.common.Access.READ)

    def update(self):
        if getattr(self.blackboard, "return_mode", False):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

