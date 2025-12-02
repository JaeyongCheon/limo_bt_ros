from modules.base_bt_nodes import (
    Node,
    Sequence,
    Fallback,
    ReactiveSequence,
    ReactiveFallback,
    Parallel,
    SyncAction,
    SyncCondition,
    AlwaysFailure,
    AlwaysSuccess,
    Status,
)

# 우리가 만든 액션 노드들 (현재는 LogMessage, Wait만 사용)
from scenarios.rescue_mission.basic_action_nodes import (
    LogMessage,
    Wait,
)


class BTNodeList:
    """
    Behavior Tree에서 사용할 노드 이름 목록
    - XML 태그 이름과 정확히 일치해야 함
    """

    # 제어 노드들 (XML에서 <Sequence>, <Fallback> 같은 것들)
    CONTROL_NODES = [
        "Sequence",
        "Fallback",
        "ReactiveSequence",
        "ReactiveFallback",
        "Parallel",
    ]

    # 액션 노드들 (리프 노드)
    # 지금은 테스트를 위해 LogMessage, Wait만 등록
    ACTION_NODES = [
        "LogMessage",
        "Wait",
    ]

    # 조건 노드들
    CONDITION_NODES = [
        "AlwaysFailure",
        "AlwaysSuccess",
    ]

    # 데코레이터 노드 (지금은 사용 안 함)
    DECORATOR_NODES = []
