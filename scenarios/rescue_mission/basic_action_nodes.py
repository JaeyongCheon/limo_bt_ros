import time
import math

from modules.base_bt_nodes import SyncAction, Status
from modules.utils import parse_target

from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose



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

        name = kwargs.get("name", node_type)

        # internal flag to ensure we only ask once
        self._asked = False
        self._node = getattr(agent, "node", None)

        def _action(agent, blackboard):
            if not self._asked:
                # Ask user in terminal
                cmd = input("Enter mission command (ex: 북쪽 3m): ").strip()
                blackboard["mission_command"] = cmd
                self._asked = True

                msg = f"[WaitForMissionCommand] Received from terminal: '{cmd}'"
                if self._node:
                    self._node.get_logger().info(msg)
                else:
                    print(msg)

                return Status.SUCCESS

            # If BT ticks this node again, we just succeed.
            return Status.SUCCESS

        super().__init__(name, _action)



#ParseTargetCommand ("북쪽 5m" → (x,y))

class ParseTargetCommand(SyncAction):
    def __init__(self, node_type, agent, **kwargs):

        name = kwargs.get("name", node_type)
        self._node = getattr(agent, "node", None)

        def _action(agent, blackboard):
            cmd = blackboard.get("mission_command", None)
            if cmd is None:
                msg = "[ParseTargetCommand] 'mission_command' not found in blackboard"
                if self._node:
                    self._node.get_logger().warn(msg)
                else:
                    print(msg)
                return Status.FAILURE

            try:
                x, y = parse_target(cmd)
            except Exception as e:
                msg = f"[ParseTargetCommand] Failed to parse '{cmd}': {e}"
                if self._node:
                    self._node.get_logger().error(msg)
                else:
                    print(msg)
                return Status.FAILURE

            blackboard["target_xy"] = (x, y)

            msg = f"[ParseTargetCommand] Parsed '{cmd}' -> x={x:.2f}, y={y:.2f}"
            if self._node:
                self._node.get_logger().info(msg)
            else:
                print(msg)
            return Status.SUCCESS

        super().__init__(name, _action)



#NavigateToTarget

class NavigateToTarget(SyncAction):
    def __init__(self, node_type, agent, **kwargs):

        name = kwargs.get("name", node_type)
        self._node = getattr(agent, "node", None)

        # Nav2 action client
        self._nav_client = None
        if self._node is not None:
            self._nav_client = ActionClient(
                self._node,
                NavigateToPose,
                "navigate_to_pose",
            )

        # Async state
        self._goal_sent = False
        self._goal_future = None
        self._result_future = None
        self._goal_handle = None

        def _action(agent, blackboard):
            # 1) Get target
            target = blackboard.get("target_xy", None)
            if target is None:
                msg = "[NavigateToTarget] 'target_xy' not found in blackboard"
                if self._node:
                    self._node.get_logger().warn(msg)
                else:
                    print(msg)
                return Status.FAILURE

            x, y = target

            # 2) If Nav2 is not available, just log and succeed (safe fallback)
            if self._nav_client is None:
                print(
                    f"[NavigateToTarget] (NO Nav2) Would move to ({x:.2f}, {y:.2f})"
                )
                return Status.SUCCESS

            # 3) Wait for action server
            if not self._nav_client.wait_for_server(timeout_sec=0.1):
                self._node.get_logger().info(
                    "[NavigateToTarget] Waiting for Nav2 action server..."
                )
                return Status.RUNNING

            # 4) Send goal once
            if not self._goal_sent:
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose = PoseStamped()
                goal_msg.pose.header.frame_id = "map"
                goal_msg.pose.header.stamp = self._node.get_clock().now().to_msg()
                goal_msg.pose.pose.position.x = float(x)
                goal_msg.pose.pose.position.y = float(y)
                goal_msg.pose.pose.orientation.w = 1.0

                self._node.get_logger().info(
                    f"[NavigateToTarget] Sending goal to x={x:.2f}, y={y:.2f}"
                )

                self._goal_future = self._nav_client.send_goal_async(goal_msg)
                self._goal_sent = True
                return Status.RUNNING

            # 5) Wait for goal handle
            if self._goal_handle is None:
                if not self._goal_future.done():
                    return Status.RUNNING

                self._goal_handle = self._goal_future.result()
                if not self._goal_handle.accepted:
                    self._node.get_logger().error(
                        "[NavigateToTarget] Goal was rejected by Nav2"
                    )
                    self._reset_nav()
                    return Status.FAILURE

                self._node.get_logger().info(
                    "[NavigateToTarget] Goal accepted, waiting for result..."
                )
                self._result_future = self._goal_handle.get_result_async()
                return Status.RUNNING

            # 6) Wait for result
            if not self._result_future.done():
                return Status.RUNNING

            result = self._result_future.result()
            status_code = result.status

            if status_code == 0:  # SUCCEEDED
                self._node.get_logger().info(
                    "[NavigateToTarget] Goal reached successfully."
                )
                self._reset_nav()
                return Status.SUCCESS
            else:
                self._node.get_logger().error(
                    f"[NavigateToTarget] Nav2 goal failed with status={status_code}"
                )
                self._reset_nav()
                return Status.FAILURE

        super().__init__(name, _action)

    def _reset_nav(self):
        self._goal_sent = False
        self._goal_future = None
        self._result_future = None
        self._goal_handle = None
        self._result_future = None


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

from geometry_msgs.msg import Twist

class RotateToAngle(SyncAction):
    def __init__(self, node_type, agent, **kwargs):
        """
        XML:
          <RotateToAngle name="Rotate90" angle_deg="90" angular_speed="0.4"/>
        """
        name = kwargs.get("name", node_type)

        self.angle_deg = float(kwargs.get("angle_deg", 90.0))
        self.angular_speed = float(kwargs.get("angular_speed", 0.4))

        self.node = agent.node
        self.pub = self.node.create_publisher(Twist, "/cmd_vel", 10)

        self.target_angle = abs(self.angle_deg) * math.pi / 180.0
        self.direction = 1 if self.angle_deg >= 0 else -1
        self.duration = self.target_angle / max(self.angular_speed, 0.001)
        self.start_time = None

        def _do(agent, blackboard):
            now = time.time()

            if self.start_time is None:
                self.start_time = now
                self.node.get_logger().info(
                    f"[RotateToAngle] Rotating {self.angle_deg:.1f}° for {self.duration:.2f}s"
                )

            elapsed = now - self.start_time

            if elapsed < self.duration:
                msg = Twist()
                msg.angular.z = self.direction * self.angular_speed
                self.pub.publish(msg)
                return Status.RUNNING

            self.pub.publish(Twist())  # stop
            self.node.get_logger().info("[RotateToAngle] Rotation complete")

            self.start_time = None
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
