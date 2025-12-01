#!/usr/bin/env python3
"""
동반 안내 모듈 - 구조대와 함께 조난자 위치로 이동
"""

import math
import numpy as np
from typing import List, Tuple, Dict, Optional


class DangerZoneAnalyzer:
    """경로상 위험 구간 분석기"""
    
    def __init__(self):
        # 위험 기준값
        self.DANGER_SLOPE_THRESHOLD = 0.3  # 약 17도
        self.DANGER_ROUGHNESS_THRESHOLD = 0.08
        self.WARNING_SLOPE_THRESHOLD = 0.2  # 약 11도 (경고)
        self.WARNING_ROUGHNESS_THRESHOLD = 0.05
    
    def identify_danger_zones(self, path: List[Tuple[float, float]], 
                             terrain_data: Dict) -> List[Dict]:
        """경로상 위험 구간 식별"""
        
        danger_zones = []
        slope = terrain_data['slope']
        roughness = terrain_data['roughness']
        resolution = terrain_data['resolution']
        
        for i, point in enumerate(path):
            # 월드 좌표를 그리드 인덱스로 변환
            grid_pos = self._world_to_grid(point, terrain_data)
            if grid_pos is None:
                continue
            
            gx, gy = grid_pos
            if not (0 <= gx < terrain_data['width'] and 0 <= gy < terrain_data['height']):
                continue
            
            slope_val = slope[gy, gx]
            roughness_val = roughness[gy, gx]
            
            # NaN 체크
            if np.isnan(slope_val) or np.isnan(roughness_val):
                continue
            
            # 위험 구간 판정
            if slope_val > self.DANGER_SLOPE_THRESHOLD:
                danger_zones.append({
                    'position': point,
                    'type': 'slope',
                    'severity': 'danger',
                    'degree': math.degrees(slope_val),
                    'index': i,
                    'description': f'급경사 {math.degrees(slope_val):.1f}도'
                })
            elif slope_val > self.WARNING_SLOPE_THRESHOLD:
                danger_zones.append({
                    'position': point,
                    'type': 'slope',
                    'severity': 'warning',
                    'degree': math.degrees(slope_val),
                    'index': i,
                    'description': f'경사 {math.degrees(slope_val):.1f}도'
                })
            
            if roughness_val > self.DANGER_ROUGHNESS_THRESHOLD:
                danger_zones.append({
                    'position': point,
                    'type': 'rough',
                    'severity': 'danger',
                    'value': roughness_val,
                    'index': i,
                    'description': f'거친 지형 (거칠기: {roughness_val:.2f})'
                })
            elif roughness_val > self.WARNING_ROUGHNESS_THRESHOLD:
                danger_zones.append({
                    'position': point,
                    'type': 'rough',
                    'severity': 'warning',
                    'value': roughness_val,
                    'index': i,
                    'description': f'약간 거친 지형 (거칠기: {roughness_val:.2f})'
                })
        
        return danger_zones
    
    def _world_to_grid(self, world_pos: Tuple[float, float], 
                      terrain_data: Dict) -> Optional[Tuple[int, int]]:
        """월드 좌표를 그리드 인덱스로 변환"""
        resolution = terrain_data['resolution']
        center_x = terrain_data['center_x']
        center_y = terrain_data['center_y']
        length_x = terrain_data['length_x']
        length_y = terrain_data['length_y']
        
        rel_x = world_pos[0] - (center_x - length_x / 2.0)
        rel_y = world_pos[1] - (center_y - length_y / 2.0)
        
        grid_x = int(rel_x / resolution)
        grid_y = int(rel_y / resolution)
        
        return (grid_x, grid_y)


class EscortMode:
    """구조대와 함께 이동하는 동반 안내 모드"""
    
    def __init__(self, logger=None):
        # 동반 이동 설정
        self.WALKING_SPEED = 1.0      # m/s (사람 보행 속도)
        self.LEAD_DISTANCE = 2.0      # 로봇이 앞서 이동하는 거리
        self.WARNING_DISTANCE = 5.0   # 위험 구간 사전 경고 거리
        self.MAX_SEPARATION = 5.0     # 최대 이격 거리 (초과 시 정지)
        
        # 위험 구간 분석기
        self.danger_analyzer = DangerZoneAnalyzer()
        
        # 로거 (없으면 print 사용)
        self.logger = logger
        
        # 상태
        self.current_waypoint_idx = 0
        self.warned_dangers = set()  # 이미 경고한 위험 구간
    
    def start_escort(self, safe_path: List[Tuple[float, float]], 
                    terrain_data: Dict,
                    victim_location: Tuple[float, float]) -> Dict:
        """동반 안내 시작
        
        Args:
            safe_path: 안전 경로 리스트 [(x1, y1), (x2, y2), ...]
            terrain_data: 지형 데이터 딕셔너리
            victim_location: 조난자 위치 (x, y)
            
        Returns:
            안내 정보 딕셔너리 (경로점, 위험구간 등)
        """
        
        self.log("🚶‍♂️🤖 구조대 동반 안내 시작!")
        self.log(f"총 경로점: {len(safe_path)}개")
        self.log(f"조난자 위치: ({victim_location[0]:.2f}, {victim_location[1]:.2f})")
        
        # 위험 구간 식별
        danger_zones = self.danger_analyzer.identify_danger_zones(safe_path, terrain_data)
        self.log(f"식별된 위험 구간: {len(danger_zones)}개")
        
        # 안내 정보 생성
        escort_info = {
            'path': safe_path,
            'danger_zones': danger_zones,
            'victim_location': victim_location,
            'total_distance': self._calculate_path_distance(safe_path),
            'estimated_time': self._estimate_travel_time(safe_path, danger_zones),
            'warnings': self._generate_warnings(danger_zones)
        }
        
        self.current_waypoint_idx = 0
        self.warned_dangers = set()
        
        return escort_info
    
    def get_next_waypoint(self, current_position: Tuple[float, float],
                         escort_info: Dict) -> Optional[Tuple[float, float]]:
        """다음 경로점 반환 및 위험 구간 경고
        
        Args:
            current_position: 현재 로봇 위치
            escort_info: start_escort()에서 반환된 정보
            
        Returns:
            다음 경로점 또는 None (경로 완료)
        """
        
        path = escort_info['path']
        
        if self.current_waypoint_idx >= len(path):
            self.log("🎯 경로 완료!")
            return None
        
        # 현재 경로점
        waypoint = path[self.current_waypoint_idx]
        
        # 위험 구간 체크
        self._check_upcoming_dangers(current_position, escort_info)
        
        # 진행률 계산
        progress = (self.current_waypoint_idx + 1) / len(path) * 100
        
        # 25% 단위로 진행 상황 알림
        if progress % 25 < (100.0 / len(path)):  # 대략 25% 근처
            self.log(f"📍 진행률: {int(progress)}%")
        
        return waypoint
    
    def advance_waypoint(self):
        """다음 경로점으로 진행"""
        self.current_waypoint_idx += 1
    
    def _check_upcoming_dangers(self, current_pos: Tuple[float, float],
                               escort_info: Dict):
        """다가오는 위험 구간 확인 및 경고"""
        
        danger_zones = escort_info['danger_zones']
        
        for danger in danger_zones:
            danger_id = danger['index']
            
            # 이미 경고한 위험 구간은 스킵
            if danger_id in self.warned_dangers:
                continue
            
            # 위험 구간까지의 거리 계산
            distance = self._calculate_distance(current_pos, danger['position'])
            
            # 경고 거리 이내이면 경고
            if distance <= self.WARNING_DISTANCE:
                severity_emoji = "⚠️" if danger['severity'] == 'warning' else "🚨"
                self.log(f"{severity_emoji} 주의: {distance:.1f}m 앞 {danger['description']}")
                self.warned_dangers.add(danger_id)
    
    def _calculate_distance(self, p1: Tuple[float, float], 
                           p2: Tuple[float, float]) -> float:
        """두 점 사이의 거리"""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    
    def _calculate_path_distance(self, path: List[Tuple[float, float]]) -> float:
        """경로 총 거리"""
        total = 0.0
        for i in range(len(path) - 1):
            total += self._calculate_distance(path[i], path[i+1])
        return total
    
    def _estimate_travel_time(self, path: List[Tuple[float, float]], 
                             danger_zones: List[Dict]) -> float:
        """예상 이동 시간 (초)"""
        distance = self._calculate_path_distance(path)
        
        # 위험 구간 수에 따라 속도 감소
        danger_count = len([d for d in danger_zones if d['severity'] == 'danger'])
        speed_factor = 1.0 - (danger_count * 0.05)  # 위험 구간마다 5% 감속
        speed_factor = max(0.5, speed_factor)  # 최소 50% 속도
        
        effective_speed = self.WALKING_SPEED * speed_factor
        return distance / effective_speed if effective_speed > 0 else float('inf')
    
    def _generate_warnings(self, danger_zones: List[Dict]) -> List[str]:
        """경고 메시지 생성"""
        warnings = []
        
        danger_count = len([d for d in danger_zones if d['severity'] == 'danger'])
        warning_count = len([d for d in danger_zones if d['severity'] == 'warning'])
        
        if danger_count > 0:
            warnings.append(f"경로상 위험 구간 {danger_count}곳 주의 필요")
        if warning_count > 0:
            warnings.append(f"경로상 주의 구간 {warning_count}곳")
        
        # 경사 위험
        steep_zones = [d for d in danger_zones if d['type'] == 'slope' and d['severity'] == 'danger']
        if steep_zones:
            max_slope = max(d['degree'] for d in steep_zones)
            warnings.append(f"최대 경사: {max_slope:.1f}도")
        
        # 거칠기 위험
        rough_zones = [d for d in danger_zones if d['type'] == 'rough' and d['severity'] == 'danger']
        if rough_zones:
            warnings.append(f"거친 지형 {len(rough_zones)}곳")
        
        return warnings
    
    def log(self, message: str):
        """로그 출력"""
        if self.logger:
            self.logger.info(message)
        else:
            print(message)
    
    def get_escort_status(self, escort_info: Dict) -> Dict:
        """현재 안내 상태 반환"""
        path = escort_info['path']
        
        return {
            'current_waypoint': self.current_waypoint_idx,
            'total_waypoints': len(path),
            'progress': (self.current_waypoint_idx / len(path) * 100) if len(path) > 0 else 0,
            'remaining_waypoints': len(path) - self.current_waypoint_idx,
            'warnings_issued': len(self.warned_dangers)
        }

