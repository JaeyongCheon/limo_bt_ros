import time
from modules.base_bt_nodes import SyncAction, Status
from modules.utils import parse_target


#LogMessage 

class LogMessage(SyncAction):
    def __init__(self, node_type, agent, **kwargs):
        """
        XML:
          <LogMessage name="StartLog" message="..." level="INFO"/>
        """
        name = kwargs.get("name", node_type)
        self.message = kwargs.get("message", "")
        self.level = kwargs.get("level", "INFO")

        def _do(agent, blackboard):
            print(f"[LogMessage][{self.level}] {self.message}")
            return Status.SUCCESS

        super().__init__(name, _do)


#Wait 

class Wait(SyncAction):
    def __init__(self, node_type, agent, **kwargs):
        """
        XML:
          <Wait name="Wait1" duration="1.5"/>
        """
        name = kwargs.get("name", node_type)
        self.duration = float(kwargs.get("duration", 1.0))
        self._start_time = None

        def _do(agent, blackboard):
            if self._start_time is None:
                self._start_time = time.time()
                print(f"[Wait] Waiting for {self.duration:.1f} seconds...")
                return Status.RUNNING

            elapsed = time.time() - self._start_time
            if elapsed < self.duration:
                return Status.RUNNING

            print("[Wait] Done waiting.")
            self._start_time = None
            return Status.SUCCESS

        super().__init__(name, _do)


# WaitForMissionCommand 

class WaitForMissionCommand(SyncAction):
    def __init__(self, node_type, agent, **kwargs):
        """
        XML:
          <WaitForMissionCommand name="WaitMission" timeout="10.0"/>
        """
        name = kwargs.get("name", node_type)
        self.timeout = float(kwargs.get("timeout", 10.0))

        def _do(agent, blackboard):
            print(f"[WaitForMissionCommand] timeout={self.timeout}s")
            if "mission_command" not in blackboard:
                blackboard["mission_command"] = "북쪽 5m"
                print("[WaitForMissionCommand] default mission_command = '북쪽 5m'")
            else:
                print(f"[WaitForMissionCommand] existing mission_command = {blackboard['mission_command']}")
            return Status.SUCCESS

        super().__init__(name, _do)


#ParseTargetCommand ("북쪽 5m" → (x,y))

class ParseTargetCommand(SyncAction):
    def __init__(self, node_type, agent, **kwargs):
        """
        XML:
          <ParseTargetCommand name="ParseTarget"/>
        """
        name = kwargs.get("name", node_type)

        def _do(agent, blackboard):
            cmd = blackboard.get("mission_command", None)
            if cmd is None:
                print("[ParseTargetCommand] 'mission_command' not in blackboard, using default '북쪽 5m'")
                cmd = "북쪽 5m"
                blackboard["mission_command"] = cmd

            try:
                x, y = parse_target(cmd)
            except Exception as e:
                print(f"[ParseTargetCommand] Failed to parse '{cmd}': {e}")
                return Status.FAILURE

            blackboard["target_xy"] = (x, y)
            print(f"[ParseTargetCommand] Parsed '{cmd}' -> x={x:.2f}, y={y:.2f}")
            return Status.SUCCESS

        super().__init__(name, _do)


#NavigateToTarget

class NavigateToTarget(SyncAction):
    def __init__(self, node_type, agent, **kwargs):
        """
        XML:
          <NavigateToTarget name="NavToTarget"/>
        """
        name = kwargs.get("name", node_type)

        def _do(agent, blackboard):
            target = blackboard.get("target_xy", None)
            if target is None:
                print("[NavigateToTarget] No 'target_xy' in blackboard")
                return Status.FAILURE

            x, y = target
            print(f"[NavigateToTarget] (TEST MODE) Pretending to move to ({x:.2f}, {y:.2f})")
            blackboard["current_pos"] = (x, y)
            return Status.SUCCESS

        super().__init__(name, _do)


#NavigateToWaypoint (경로점 이동)

class NavigateToWaypoint(SyncAction):
    def __init__(self, node_type, agent, **kwargs):
        """
        XML:
          <NavigateToWaypoint name="NavToWP"/>
        """
        name = kwargs.get("name", node_type)

        def _do(agent, blackboard):
            waypoints = blackboard.get("waypoints", None)
            if not waypoints:
                waypoints = [(1.0, 0.0), (1.0, 1.0)]
                blackboard["waypoints"] = waypoints
                blackboard["waypoint_index"] = 0
                print(f"[NavigateToWaypoint] default waypoints = {waypoints}")

            idx = blackboard.get("waypoint_index", 0)
            if idx >= len(waypoints):
                print("[NavigateToWaypoint] All waypoints used.")
                return Status.SUCCESS

            x, y = waypoints[idx]
            blackboard["target_xy"] = (x, y)
            blackboard["waypoint_index"] = idx + 1
            print(f"[NavigateToWaypoint] Using waypoint #{idx}: ({x:.2f}, {y:.2f})")
            return Status.SUCCESS

        super().__init__(name, _do)


#RotateToAngle

class RotateToAngle(SyncAction):
    def __init__(self, node_type, agent, **kwargs):
        """
        XML:
          <RotateToAngle name="Rotate90" angle_deg="90" angular_speed="0.5"/>
        """
        name = kwargs.get("name", node_type)
        self.angle_deg = float(kwargs.get("angle_deg", 90.0))
        self.angular_speed = float(kwargs.get("angular_speed", 0.5))
        self._done = False

        def _do(agent, blackboard):
            if not self._done:
                print(f"[RotateToAngle] (TEST MODE) Rotating {self.angle_deg:.1f} deg at {self.angular_speed:.2f} rad/s")
                self._done = True
                return Status.RUNNING
            print("[RotateToAngle] Rotation complete.")
            self._done = False
            return Status.SUCCESS

        super().__init__(name, _do)


#ConfirmVictimLocation

class ConfirmVictimLocation(SyncAction):
    def __init__(self, node_type, agent, **kwargs):
        """
        XML:
          <ConfirmVictimLocation name="ConfirmVictim"/>
        """
        name = kwargs.get("name", node_type)

        def _do(agent, blackboard):
            victim_loc = blackboard.get("victim_location", None)
            if victim_loc is None:
                victim_loc = (2.0, 3.0)
                blackboard["victim_location"] = victim_loc
                print("[ConfirmVictimLocation] default victim_location set to (2.0, 3.0)")
            print(f"[ConfirmVictimLocation] Victim location confirmed: {victim_loc}")
            return Status.SUCCESS

        super().__init__(name, _do)


#StartClueMonitoring

class StartClueMonitoring(SyncAction):
    def __init__(self, node_type, agent, **kwargs):
        """
        XML:
          <StartClueMonitoring name="StartClues"/>
        """
        name = kwargs.get("name", node_type)

        def _do(agent, blackboard):
            blackboard["clue_monitoring_enabled"] = True
            if "clues" not in blackboard:
                blackboard["clues"] = ["clue A", "clue B"]
            blackboard["clue_index"] = 0
            print("[StartClueMonitoring] Clue monitoring enabled. clues =", blackboard["clues"])
            return Status.SUCCESS

        super().__init__(name, _do)


#GetNextClue

class GetNextClue(SyncAction):
    def __init__(self, node_type, agent, **kwargs):
        """
        XML:
          <GetNextClue name="NextClue"/>
        """
        name = kwargs.get("name", node_type)

        def _do(agent, blackboard):
            clues = blackboard.get("clues", [])
            idx = blackboard.get("clue_index", 0)

            if not clues:
                print("[GetNextClue] No clues available.")
                return Status.FAILURE

            if idx >= len(clues):
                print("[GetNextClue] All clues consumed.")
                return Status.SUCCESS

            clue = clues[idx]
            blackboard["current_clue"] = clue
            blackboard["clue_index"] = idx + 1
            print(f"[GetNextClue] Using clue #{idx}: {clue}")
            return Status.SUCCESS

        super().__init__(name, _do)
