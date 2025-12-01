#!/usr/bin/env python3
"""
경로 계획 모듈 - 구조 로봇용 이중 경로 생성
GridMap 기반으로 로봇용/사람용 경로를 각각 계획
"""

import math
import heapq
import numpy as np
from typing import List, Tuple, Dict, Optional


class DualPathPlanner:
    """로봇용/구조대용 이중 경로 생성"""
    
    def __init__(self):
        # 사람 기준 안전 기준
        self.MAX_HUMAN_SLOPE = 0.52  # 약 30도 (라디안)
        self.MAX_HUMAN_ROUGHNESS = 0.1
        
        # 경로 전략별 가중치
        self.strategy_weights = {
            'safest': {'distance': 0.2, 'safety': 0.8},
            'balanced': {'distance': 0.5, 'safety': 0.5},
            'shortest': {'distance': 0.9, 'safety': 0.1}
        }
    
    def generate_rescue_paths(self, start: Tuple[float, float], 
                             victim_loc: Tuple[float, float], 
                             grid_map) -> List[Dict]:
        """구조대를 위한 3가지 경로 생성"""
        
        # GridMap에서 데이터 추출
        terrain_data = self._extract_terrain_data(grid_map)
        
        # 사람용 Costmap 생성
        human_costmap = self.create_human_costmap(terrain_data)
        
        paths = []
        
        # 경로 1: 최대 안전
        safe_path = self.plan_path(
            start, victim_loc, human_costmap, terrain_data,
            strategy='safest'
        )
        if safe_path:
            paths.append({
                'name': '안전 경로',
                'path': safe_path,
                'metrics': self.evaluate_path(safe_path, human_costmap, terrain_data),
                'recommendation': '일반 구조대 추천'
            })
        
        # 경로 2: 균형
        balanced_path = self.plan_path(
            start, victim_loc, human_costmap, terrain_data,
            strategy='balanced'
        )
        if balanced_path:
            paths.append({
                'name': '균형 경로',
                'path': balanced_path,
                'metrics': self.evaluate_path(balanced_path, human_costmap, terrain_data),
                'recommendation': '숙련된 구조대'
            })
        
        # 경로 3: 최단
        shortest_path = self.plan_path(
            start, victim_loc, human_costmap, terrain_data,
            strategy='shortest'
        )
        if shortest_path:
            paths.append({
                'name': '최단 경로',
                'path': shortest_path,
                'metrics': self.evaluate_path(shortest_path, human_costmap, terrain_data),
                'recommendation': '전문 산악구조대만'
            })
        
        return paths
    
    def _extract_terrain_data(self, grid_map) -> Dict:
        """GridMap에서 지형 데이터 추출"""
        
        terrain_data = {
            'resolution': grid_map.info.resolution,
            'length_x': grid_map.info.length_x,
            'length_y': grid_map.info.length_y,
            'center_x': grid_map.info.pose.position.x,
            'center_y': grid_map.info.pose.position.y,
        }
        
        width = int(terrain_data['length_x'] / terrain_data['resolution'])
        height = int(terrain_data['length_y'] / terrain_data['resolution'])
        
        terrain_data['width'] = width
        terrain_data['height'] = height
        
        # 레이어 데이터 추출
        for layer_name in ['elevation', 'slope', 'roughness', 'traversability']:
            if layer_name in grid_map.layers:
                layer_idx = grid_map.layers.index(layer_name)
                layer_data = np.array(grid_map.data[layer_idx].data).reshape((height, width))
                terrain_data[layer_name] = layer_data
            else:
                terrain_data[layer_name] = np.zeros((height, width))
        
        return terrain_data
    
    def create_human_costmap(self, terrain_data: Dict) -> np.ndarray:
        """사람 기준의 비용 지도 생성"""
        
        height = terrain_data['height']
        width = terrain_data['width']
        slope = terrain_data['slope']
        roughness = terrain_data['roughness']
        
        human_costmap = np.zeros((height, width), dtype=np.float32)
        
        for y in range(height):
            for x in range(width):
                slope_val = slope[y, x]
                roughness_val = roughness[y, x]
                
                # NaN 처리
                if np.isnan(slope_val) or np.isnan(roughness_val):
                    human_costmap[y, x] = 255  # 미지 영역은 통과 불가
                    continue
                
                cost = 0
                
                # 경사 비용
                if slope_val > 0.5:
                    cost = 255  # 통과 불가
                elif slope_val > 0.3:
                    cost += 200  # 매우 위험
                elif slope_val > 0.2:
                    cost += 100  # 위험
                else:
                    cost += int(slope_val * 200)
                
                # 거칠기 비용
                if roughness_val > 0.2:
                    cost = 255  # 통과 불가
                elif roughness_val > 0.1:
                    cost += 100
                else:
                    cost += int(roughness_val * 500)
                
                human_costmap[y, x] = min(cost, 255)
        
        return human_costmap
    
    def plan_path(self, start: Tuple[float, float], goal: Tuple[float, float],
                  costmap: np.ndarray, terrain_data: Dict, 
                  strategy: str = 'balanced') -> Optional[List[Tuple[float, float]]]:
        """A* 알고리즘으로 경로 생성"""
        
        # 월드 좌표를 그리드 인덱스로 변환
        start_grid = self._world_to_grid(start, terrain_data)
        goal_grid = self._world_to_grid(goal, terrain_data)
        
        if start_grid is None or goal_grid is None:
            return None
        
        # 범위 체크
        height, width = costmap.shape
        if not (0 <= start_grid[0] < width and 0 <= start_grid[1] < height):
            return None
        if not (0 <= goal_grid[0] < width and 0 <= goal_grid[1] < height):
            return None
        
        # 전략별 가중치
        weights = self.strategy_weights[strategy]
        
        # A* 알고리즘
        open_set = []
        heapq.heappush(open_set, (0, start_grid))
        
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: self._heuristic(start_grid, goal_grid)}
        
        # 8방향 이동
        directions = [
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            # 목표 도달
            if current == goal_grid:
                path_grid = self._reconstruct_path(came_from, current)
                # 그리드 좌표를 월드 좌표로 변환
                path_world = [self._grid_to_world(p, terrain_data) for p in path_grid]
                return path_world
            
            # 이웃 탐색
            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # 범위 체크
                if not (0 <= neighbor[0] < width and 0 <= neighbor[1] < height):
                    continue
                
                # Costmap 비용
                cost = costmap[neighbor[1], neighbor[0]]
                
                # 통과 불가 (255)
                if cost >= 254:
                    continue
                
                # 이동 비용 계산
                move_cost = math.sqrt(dx*dx + dy*dy)
                terrain_cost = cost / 255.0
                
                # 전략별 가중치 적용
                tentative_g = g_score[current] + (
                    weights['distance'] * move_cost + 
                    weights['safety'] * terrain_cost
                )
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + weights['distance'] * self._heuristic(neighbor, goal_grid)
                    f_score[neighbor] = f
                    heapq.heappush(open_set, (f, neighbor))
        
        # 경로를 찾지 못함
        return None
    
    def _heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """유클리드 거리 휴리스틱"""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def _reconstruct_path(self, came_from: Dict, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """경로 재구성"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    
    def _world_to_grid(self, world_pos: Tuple[float, float], terrain_data: Dict) -> Optional[Tuple[int, int]]:
        """월드 좌표를 그리드 인덱스로 변환"""
        resolution = terrain_data['resolution']
        center_x = terrain_data['center_x']
        center_y = terrain_data['center_y']
        length_x = terrain_data['length_x']
        length_y = terrain_data['length_y']
        
        # 상대 위치 계산
        rel_x = world_pos[0] - (center_x - length_x / 2.0)
        rel_y = world_pos[1] - (center_y - length_y / 2.0)
        
        grid_x = int(rel_x / resolution)
        grid_y = int(rel_y / resolution)
        
        return (grid_x, grid_y)
    
    def _grid_to_world(self, grid_pos: Tuple[int, int], terrain_data: Dict) -> Tuple[float, float]:
        """그리드 인덱스를 월드 좌표로 변환"""
        resolution = terrain_data['resolution']
        center_x = terrain_data['center_x']
        center_y = terrain_data['center_y']
        length_x = terrain_data['length_x']
        length_y = terrain_data['length_y']
        
        world_x = (center_x - length_x / 2.0) + grid_pos[0] * resolution + resolution / 2.0
        world_y = (center_y - length_y / 2.0) + grid_pos[1] * resolution + resolution / 2.0
        
        return (world_x, world_y)
    
    def evaluate_path(self, path: List[Tuple[float, float]], 
                     costmap: np.ndarray, terrain_data: Dict) -> Dict:
        """경로 품질 평가"""
        
        total_distance = 0.0
        total_risk = 0.0
        max_slope = 0.0
        max_roughness = 0.0
        
        slope = terrain_data['slope']
        roughness = terrain_data['roughness']
        resolution = terrain_data['resolution']
        
        for i in range(len(path) - 1):
            p1, p2 = path[i], path[i+1]
            
            # 거리
            segment_dist = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
            total_distance += segment_dist
            
            # 위험도 (costmap 비용)
            grid_pos = self._world_to_grid(p2, terrain_data)
            if grid_pos:
                gx, gy = grid_pos
                if 0 <= gx < terrain_data['width'] and 0 <= gy < terrain_data['height']:
                    cost = costmap[gy, gx]
                    total_risk += cost
                    
                    # 최대값 추적
                    slope_val = slope[gy, gx]
                    roughness_val = roughness[gy, gx]
                    
                    if not np.isnan(slope_val):
                        max_slope = max(max_slope, slope_val)
                    if not np.isnan(roughness_val):
                        max_roughness = max(max_roughness, roughness_val)
        
        # 예상 시간 계산
        walking_speed = 1.0  # m/s (사람 보행 속도)
        avg_risk = total_risk / len(path) if len(path) > 0 else 0
        speed_reduction = 1.0 - (avg_risk / 255) * 0.5
        effective_speed = walking_speed * speed_reduction
        estimated_time = total_distance / effective_speed if effective_speed > 0 else float('inf')
        
        return {
            'distance': total_distance,
            'time': estimated_time,
            'risk_score': avg_risk,
            'max_slope': max_slope,
            'max_roughness': max_roughness,
            'difficulty': self.calculate_difficulty(avg_risk)
        }
    
    def calculate_difficulty(self, avg_risk: float) -> str:
        """위험도를 별점으로 변환"""
        if avg_risk < 30:
            return '★☆☆☆☆ (쉬움)'
        elif avg_risk < 60:
            return '★★☆☆☆ (보통)'
        elif avg_risk < 90:
            return '★★★☆☆ (어려움)'
        elif avg_risk < 120:
            return '★★★★☆ (매우 어려움)'
        else:
            return '★★★★★ (극한)'


def generate_spiral_waypoints(center: Tuple[float, float], max_radius: float, 
                              angular_step: float = math.pi / 4, 
                              radial_step: float = 0.3) -> List[Tuple[float, float]]:
    """나선형 경로점 생성"""
    waypoints = []
    angle = 0.0
    radius = 0.5
    
    while radius <= max_radius:
        x = center[0] + radius * math.cos(angle)
        y = center[1] + radius * math.sin(angle)
        waypoints.append((x, y))
        
        angle += angular_step  # 기본 45도씩 회전
        radius += radial_step  # 기본 30cm씩 확장
    
    return waypoints

