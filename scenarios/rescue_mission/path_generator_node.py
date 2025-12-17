#!/usr/bin/env python3
# path_generator_node.py
#
# 역할: 위험도 맵 + 시작/목표로부터 A* 경로 3개 생성 (안전, 균형, 최단)
#
# 구독:
#  - /risk_map (nav_msgs/msg/OccupancyGrid)
#  - /start_pose (geometry_msgs/msg/PoseStamped)
#  - /victim_pose (geometry_msgs/msg/PoseStamped)
#
# 퍼블리시:
#  - /paths/safe (nav_msgs/msg/Path)
#  - /paths/balanced (nav_msgs/msg/Path)
#  - /paths/shortest (nav_msgs/msg/Path)
#  - /paths/info (std_msgs/msg/String)  # 거리/시간/위험도 간단 요약

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import numpy as np
import heapq
import math
import time

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')
        self.declare_parameter('resolution', 0.5)
        self.resolution = float(self.get_parameter('resolution').get_parameter_value().double_value)
        self.risk_map = None
        self.map_info = None
        self.start = None
        self.goal = None

        self.create_subscription(OccupancyGrid, '/risk_map', self.risk_cb, 1)
        self.create_subscription(PoseStamped, '/start_pose', self.start_cb, 1)
        self.create_subscription(PoseStamped, '/victim_pose', self.goal_cb, 1)

        self.safe_pub = self.create_publisher(Path, '/paths/safe', 1)
        self.bal_pub = self.create_publisher(Path, '/paths/balanced', 1)
        self.short_pub = self.create_publisher(Path, '/paths/shortest', 1)
        self.info_pub = self.create_publisher(String, '/paths/info', 1)

        self.create_timer(1.0, self.try_plan)

    def risk_cb(self, msg: OccupancyGrid):
        self.risk_map = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    def start_cb(self, msg: PoseStamped):
        self.start = (msg.pose.position.x, msg.pose.position.y)

    def goal_cb(self, msg: PoseStamped):
        self.goal = (msg.pose.position.x, msg.pose.position.y)

    def idx_from_xy(self, x, y):
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        res = self.map_info.resolution
        ix = int((x - ox) / res)
        iy = int((y - oy) / res)
        if 0 <= ix < self.map_info.width and 0 <= iy < self.map_info.height:
            return ix, iy
        return None

    def xy_from_idx(self, ix, iy):
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        res = self.map_info.resolution
        x = ox + (ix + 0.5) * res
        y = oy + (iy + 0.5) * res
        return x, y

    def try_plan(self):
        if self.risk_map is None or self.start is None or self.goal is None:
            return
        # convert start/goal to grid indices
        s_idx = self.idx_from_xy(self.start[0], self.start[1])
        g_idx = self.idx_from_xy(self.goal[0], self.goal[1])
        if s_idx is None or g_idx is None:
            self.get_logger().warning('Start or goal out of map bounds')
            return

        # generate three paths by alpha weighting (risk penalty)
        alphas = {'safe': 50.0, 'balanced': 5.0, 'shortest': 0.0}
        results = {}
        for name, alpha in alphas.items():
            path_idx, cost, risk = self.astar(s_idx, g_idx, alpha)
            if path_idx is None:
                self.get_logger().warning(f'No path found for {name}')
                results[name] = None
            else:
                # convert to nav_msgs/Path
                pmsg = Path()
                pmsg.header.frame_id = self.map_info.origin.position.x and 'map' or 'map'
                pmsg.header.stamp = self.get_clock().now().to_msg()
                for (ix, iy) in path_idx:
                    x, y = self.xy_from_idx(ix, iy)
                    ps = PoseStamped()
                    ps.header = pmsg.header
                    ps.pose.position.x = x
                    ps.pose.position.y = y
                    ps.pose.orientation.w = 1.0
                    pmsg.poses.append(ps)
                results[name] = (pmsg, cost, risk)

        # publish results
        if results['safe']:
            self.safe_pub.publish(results['safe'][0])
        if results['balanced']:
            self.bal_pub.publish(results['balanced'][0])
        if results['shortest']:
            self.short_pub.publish(results['shortest'][0])

        # publish summary
        summary = []
        for k in ['safe','balanced','shortest']:
            v = results[k]
            if v:
                summary.append(f"{k}: dist={v[1]:.1f} risk={v[2]:.2f}")
            else:
                summary.append(f"{k}: none")
        s = String()
        s.data = ' | '.join(summary)
        self.info_pub.publish(s)

    def astar(self, start, goal, alpha):
        # A* on grid. cost = euclidean_dist + alpha * (risk normalized)
        h, w = self.risk_map.shape
        sx, sy = start
        gx, gy = goal

        # priority queue
        open_set = []
        heapq.heappush(open_set, (0.0, (sx, sy)))
        came_from = {}
        gscore = { (sx, sy): 0.0 }
        fscore = { (sx, sy): self.heuristic((sx,sy), (gx,gy)) }

        # 8-neighbors
        neighbors = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

        max_iter = h*w*10
        it = 0
        while open_set and it < max_iter:
            it += 1
            _, current = heapq.heappop(open_set)
            if current == (gx, gy):
                # reconstruct
                path = []
                cur = current
                total_risk = 0.0
                while cur in came_from:
                    path.append(cur)
                    total_risk += self.risk_map[cur[1], cur[0]] if self.risk_map is not None else 0
                    cur = came_from[cur]
                path.append((sx,sy))
                path.reverse()
                # distance estimate in meters = grid steps * resolution
                dist = gscore[(gx,gy)]
                avg_risk = total_risk / max(1, len(path))
                return path, dist*self.map_info.resolution, avg_risk/100.0

            for dx,dy in neighbors:
                nx = current[0] + dx
                ny = current[1] + dy
                if nx < 0 or ny < 0 or nx >= self.map_info.width or ny >= self.map_info.height:
                    continue
                base_cost = math.hypot(dx, dy) * self.map_info.resolution
                # risk penalty: normalized 0..1 from occupancy value (0..100)
                risk_val = float(self.risk_map[ny, nx]) / 100.0
                tentative = gscore.get(current, 1e9) + base_cost + alpha * risk_val
                if tentative < gscore.get((nx,ny), 1e9):
                    came_from[(nx,ny)] = current
                    gscore[(nx,ny)] = tentative
                    f = tentative + self.heuristic((nx,ny),(gx,gy))
                    heapq.heappush(open_set, (f, (nx,ny)))
        return None, None, None

    def heuristic(self, a, b):
        # Euclidean in grid cells scaled to meters
        return math.hypot(a[0]-b[0], a[1]-b[1]) * self.map_info.resolution

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
