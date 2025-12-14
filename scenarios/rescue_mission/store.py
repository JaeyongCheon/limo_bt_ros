import math
import asyncio
import hashlib
import uuid
from collections import deque

from modules.base_bt_nodes import Node, Status
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped


class SaveVictimAndPath(Node):
    
    def __init__(self, name, agent, max_path_len=2000, min_sample_dist=0.2, target_frame="map"):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.type = "Action"

        self.max_path_len = max_path_len
        self.min_sample_dist = min_sample_dist
        self.target_frame = target_frame

    async def run(self, agent, blackboard):
        try:
            lock = blackboard.get("_lock", None)
            # obtain odom safely
            odom = blackboard.get("odom", None)
            if odom is None:
                self.ros.node.get_logger().debug(f"{self.name}: odom not available on blackboard.")
                self.status = Status.FAILURE
                return self.status

            # try transform to target frame if tf_buffer present and frames differ
            current_pos = None
            try:
                frame_id = getattr(odom, "header", None) and getattr(odom.header, "frame_id", None)
                if frame_id and frame_id != self.target_frame and "tf_buffer" in blackboard:
                    tf_buffer = blackboard["tf_buffer"]
                    # attempt transform if tf_buffer has transform API
                    if hasattr(tf_buffer, "transform"):
                        pose_stamped = PoseStamped()
                        pose_stamped.header = odom.header
                        pose_stamped.pose = odom.pose.pose
                        try:
                            trans = tf_buffer.transform(pose_stamped, self.target_frame, timeout=0.1)
                            current_pos = (trans.pose.position.x, trans.pose.position.y)
                        except Exception as e:
                            # fallback to raw odom pose (but log)
                            self.ros.node.get_logger().warn(f"{self.name}: TF transform failed: {e}. Using raw odom frame.")
                            current_pos = (odom.pose.pose.position.x, odom.pose.pose.position.y)
                    else:
                        current_pos = (odom.pose.pose.position.x, odom.pose.pose.position.y)
                else:
                    current_pos = (odom.pose.pose.position.x, odom.pose.pose.position.y)
            except Exception as e:
                self.ros.node.get_logger().error(f"{self.name}: error extracting position from odom: {e}")
                self.status = Status.FAILURE
                return self.status

            victim_detected = blackboard.get("victim_detected", False)
            if not victim_detected:
                self.status = Status.FAILURE
                return self.status

            # Lock if available (supports asyncio.Lock or threading.Lock)
            if lock:
                # support both sync and async locks
                if hasattr(lock, "acquire") and asyncio.iscoroutinefunction(lock.acquire):
                    await lock.acquire()
                    try:
                        self._save_path_and_victim(blackboard, current_pos)
                    finally:
                        lock.release()
                else:
                    # assume threading.Lock
                    lock.acquire()
                    try:
                        self._save_path_and_victim(blackboard, current_pos)
                    finally:
                        lock.release()
            else:
                self._save_path_and_victim(blackboard, current_pos)

            self.status = Status.SUCCESS
            return self.status
        except Exception as e:
            self.ros.node.get_logger().error(f"{self.name}: unexpected error: {e}")
            self.status = Status.FAILURE
            return self.status

    def _save_path_and_victim(self, blackboard, current_pos):
        # initialize visited_path as deque if not present
        if "visited_path" not in blackboard or not isinstance(blackboard["visited_path"], deque):
            blackboard["visited_path"] = deque(maxlen=self.max_path_len)

        visited = blackboard["visited_path"]
        # sample by distance to last point
        last = visited[-1] if len(visited) > 0 else None
        if last is None or self._distance(last, current_pos) >= self.min_sample_dist:
            visited.append(current_pos)

        # victim_location 저장 (한번만 저장하려면 기존이 없을 때만 저장)
        # 여기서는 처음 발견된 위치를 우선 저장 (추가 로직 원하면 수정)
        if "victim_location" not in blackboard:
            blackboard["victim_location"] = current_pos

        self.ros.node.get_logger().info(
            f"👤 [{self.name}] victim saved at {blackboard.get('victim_location')}, path_len={len(visited)}"
        )

    @staticmethod
    def _distance(a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return math.hypot(dx, dy)


class VisualizeVisitedPath(Node):
    
    def __init__(self, name, agent, topic="/visited_path_markers", max_points=800, target_frame="map"):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.marker_pub = self.ros.node.create_publisher(MarkerArray, topic, 10)
        self.type = "Action"

        self.prev_hash = None
        self.prev_marker_count = 0
        self.max_points = max_points
        self.target_frame = target_frame
        self._uid = uuid.uuid4().hex[:8]  # unique suffix to reduce chance of ns collision

    async def run(self, agent, blackboard):
        try:
            path = list(blackboard.get("visited_path", []))
            victim_location = blackboard.get("victim_location", None)

            if not path and not victim_location:
                # nothing to show; if previously published markers exist, delete them
                if self.prev_marker_count > 0:
                    self._publish_delete_all(self.prev_marker_count)
                    self.prev_marker_count = 0
                    self.prev_hash = None
                self.status = Status.FAILURE
                return self.status

            # compute a quick hash to detect changes (order + coords + victim)
            digest = self._compute_hash(path, victim_location)
            if digest == self.prev_hash:
                # no change -> avoid republishing
                self.status = Status.SUCCESS
                return self.status

            # downsample path if too many points
            ds_path = self._downsample_path(path, self.max_points)

            marker_array = MarkerArray()
            marker_id = 0
            now = self.ros.node.get_clock().now().to_msg()

            # LINE_STRIP for path
            if ds_path:
                line_marker = Marker()
                line_marker.header.frame_id = self.target_frame
                line_marker.header.stamp = now
                line_marker.ns = f"visited_path_{self._uid}"
                line_marker.id = marker_id
                marker_id += 1
                line_marker.type = Marker.LINE_STRIP
                line_marker.action = Marker.ADD
                line_marker.scale.x = 0.08  # line width
                # set color (use rgba fields)
                line_marker.color.r = 0.0
                line_marker.color.g = 0.0
                line_marker.color.b = 1.0
                line_marker.color.a = 0.8

                for pos in ds_path:
                    p = Point()
                    p.x, p.y, p.z = pos[0], pos[1], 0.0
                    line_marker.points.append(p)

                marker_array.markers.append(line_marker)

            # Victim marker
            if victim_location:
                victim_marker = Marker()
                victim_marker.header.frame_id = self.target_frame
                victim_marker.header.stamp = now
                victim_marker.ns = f"victim_{self._uid}"
                victim_marker.id = marker_id
                marker_id += 1
                victim_marker.type = Marker.SPHERE
                victim_marker.action = Marker.ADD
                victim_marker.pose.position.x = victim_location[0]
                victim_marker.pose.position.y = victim_location[1]
                victim_marker.pose.position.z = 0.5
                victim_marker.scale.x = 0.5
                victim_marker.scale.y = 0.5
                victim_marker.scale.z = 0.5
                victim_marker.color.r = 1.0
                victim_marker.color.g = 0.0
                victim_marker.color.b = 0.0
                victim_marker.color.a = 1.0

                marker_array.markers.append(victim_marker)

            # before publishing new markers, if previous had more markers, publish deletes for leftovers
            if self.prev_marker_count > marker_id:
                # publish deletes for ids in range(marker_id, prev_marker_count)
                for old_id in range(marker_id, self.prev_marker_count):
                    del_marker = Marker()
                    del_marker.header.frame_id = self.target_frame
                    del_marker.header.stamp = now
                    del_marker.ns = f"visited_path_{self._uid}"
                    del_marker.id = old_id
                    del_marker.action = Marker.DELETE
                    marker_array.markers.append(del_marker)

            # publish
            self.marker_pub.publish(marker_array)
            self.ros.node.get_logger().info(
                f"🗺️ [{self.name}] Published {marker_id} markers (path_points={len(ds_path)}, victim={victim_location})"
            )

            self.prev_marker_count = marker_id
            self.prev_hash = digest
            self.status = Status.SUCCESS
            return self.status

        except Exception as e:
            self.ros.node.get_logger().error(f"{self.name}: visualization error: {e}")
            self.status = Status.FAILURE
            return self.status

    def _compute_hash(self, path, victim):
        m = hashlib.sha256()
        for p in path:
            m.update(f"{p[0]:.4f},{p[1]:.4f};".encode())
        if victim:
            m.update(f"V:{victim[0]:.4f},{victim[1]:.4f}".encode())
        return m.hexdigest()

    def _downsample_path(self, path, max_points):
        n = len(path)
        if n <= max_points:
            return path
        # uniform downsample
        step = n / max_points
        return [path[int(i * step)] for i in range(max_points)]

    def _publish_delete_all(self, count):
        # publish DELETE markers to remove previous markers
        marker_array = MarkerArray()
        now = self.ros.node.get_clock().now().to_msg()
        for i in range(count):
            del_marker = Marker()
            del_marker.header.frame_id = self.target_frame
            del_marker.header.stamp = now
            del_marker.ns = f"visited_path_{self._uid}"
            del_marker.id = i
            del_marker.action = Marker.DELETE
            marker_array.markers.append(del_marker)
        self.marker_pub.publish(marker_array)
        self.ros.node.get_logger().info(f"[{self.name}] Published DELETE for {count} markers.")
