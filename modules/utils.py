import yaml
import os
import xml.etree.ElementTree as ET
import importlib
import math
# Global configuration and environment package
config = None
env_pkg = "rescue_mission"  # 기본 시나리오 패키지 이름


def load_config(config_file):
    with open(config_file, 'r', encoding="utf-8") as f:
        return yaml.safe_load(f)


def set_config(config_file):
    """
    YAML 설정 파일을 읽어서 전역 config와 env_pkg를 설정
    """
    global config, env_pkg
    config = load_config(config_file)
    config['config_file_path'] = config_file

    # config.yaml 안에 env_pkg가 있으면 그것을 우선 사용
    if 'env_pkg' in config:
        env_pkg = config['env_pkg']


def get_file_dirname(file):
    # 모듈 파일 기준 디렉토리 반환
    return os.path.dirname(os.path.abspath(file))


def parse_behavior_tree(xml_path):
    tree = ET.parse(xml_path)
    root = tree.getroot()
    return root


def convert_value(v):
    """
    "None" → None
    문자열 숫자는 int/float로 변환, 나머지는 그대로
    """
    if v == "None":
        return None
    if isinstance(v, str):
        # 정수
        if v.isdigit() or (v.startswith('-') and v[1:].isdigit()):
            return int(v)
        # 실수
        try:
            return float(v)
        except ValueError:
            pass
    return v


def optional_import(name):
    """
    문자열로부터 모듈 임포트 시도.
    - name이 None/빈 문자열이면 None 반환
    - 모듈이 아예 없으면 None 반환
    - 내부 의존성 문제는 예외 그대로 올림
    """
    if not name:
        return None
    try:
        return importlib.import_module(name)
    except ModuleNotFoundError as e:
        # 요청한 모듈 자체가 없을 때만 None 반환
        if e.name == name:
            return None
        # 내부 의존 모듈 누락 등은 그대로 예외 발생
        raise
def parse_target(command: str):
    """
    Simple direction+distance parser.
    Example:
      "북쪽 5m" -> (x, y)
    """
    direction_angles = {
        '북쪽': 90, '남쪽': 270,
        '동쪽': 0,  '서쪽': 180,
        '북동쪽': 45,  '남동쪽': 315,
        '북서쪽': 135, '남서쪽': 225,
    }

    parts = command.split()
    if len(parts) < 2:
        return 0.0, 0.0

    direction = parts[0]
    dist_str = parts[1]

    try:
        distance = float(dist_str.replace("m", "").replace("M", ""))
    except Exception:
        distance = 0.0

    angle_deg = direction_angles.get(direction, 0)
    angle_rad = math.radians(angle_deg)

    x = distance * math.cos(angle_rad)
    y = distance * math.sin(angle_rad)

    return x, y
