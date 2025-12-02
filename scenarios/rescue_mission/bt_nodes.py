from modules.base_bt_nodes import Sequence, Fallback
from .basic_action_nodes import (
    LogMessage,
    Wait,
    WaitForMissionCommand,
    ParseTargetCommand,
    NavigateToTarget,
    NavigateToWaypoint,
    RotateToAngle,
    ConfirmVictimLocation,
    StartClueMonitoring,
    GetNextClue,
)


class BTNodeList:
    # 컨트롤 노드들 (XML에서 <Sequence>, <Fallback> 등)
    CONTROL_NODES = [
        "Sequence",
        "Fallback",
    ]

    # 데코레이터 노드가 있으면 여기에 이름을 추가
    DECORATOR_NODES = [
        # 예: "Inverter", "RetryUntilSuccessful"
    ]

    # 액션 노드들 (우리가 basic_action_nodes.py 에 정의한 것들)
    ACTION_NODES = [
        "LogMessage",
        "Wait",
        "WaitForMissionCommand",
        "ParseTargetCommand",
        "NavigateToTarget",
        "NavigateToWaypoint",
        "RotateToAngle",
        "ConfirmVictimLocation",
        "StartClueMonitoring",
        "GetNextClue",
    ]

    # 조건 노드 사용하면 여기에 추가
    CONDITION_NODES = [
        # 예: "IsBatteryLow"
    ]


# 선택적: 이름 → 클래스 매핑 (직접 쓸 일 있으면 사용)
NODE_TYPES = {
    "LogMessage": LogMessage,
    "Wait": Wait,
    "WaitForMissionCommand": WaitForMissionCommand,
    "ParseTargetCommand": ParseTargetCommand,
    "NavigateToTarget": NavigateToTarget,
    "NavigateToWaypoint": NavigateToWaypoint,
    "RotateToAngle": RotateToAngle,
    "ConfirmVictimLocation": ConfirmVictimLocation,
    "StartClueMonitoring": StartClueMonitoring,
    "GetNextClue": GetNextClue,
    "Sequence": Sequence,
    "Fallback": Fallback,
}

