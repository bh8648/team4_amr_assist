#!/usr/bin/env python3
"""
vision_utils.py

oakd_detector_node가 쓰는 순수 함수 모음. rclpy에 의존하지 않아 노드 없이 단위 검증이 가능하다.

depth 디코딩/샘플링은 이전 워크스페이스(rc_car_chase)에서 실기 검증된 구현을 이 패키지 소유로
가져온 것이고, 역투영/발끝 추정은 이 노드를 위해 새로 작성했다.
"""

import math

import cv2
import numpy as np


# COCO 17 keypoint 인덱스 (YOLO-pose 출력 순서)
KP_LEFT_KNEE = 13
KP_RIGHT_KNEE = 14
KP_LEFT_ANKLE = 15
KP_RIGHT_ANKLE = 16

# 발끝 좌표 산출 방식별 위치 표준편차 (m).
# 문서 4번의 3등급(각도만 추정 / 무릎 keypoint로 보정 / 발끝 직접 검출)에 대응하며,
# Detection3D.results[].pose.covariance로 실려 하위 칼만필터의 관측 노이즈가 된다.
# 실측 캘리브레이션 값이 아니라 등급 간 상대 신뢰도를 표현하기 위한 초기값이다.
GRADE_SIGMA = {
    'toe_direct': 0.05,
    'knee_corrected': 0.15,
    'angle_only': 0.30,
}

# 등급별 신뢰도 가중치. Detection3D.results[1].hypothesis.score로 실린다.
GRADE_SCORE = {
    'toe_direct': 1.0,
    'knee_corrected': 0.6,
    'angle_only': 0.3,
}

# base_link 기준 IR 근접센서 방위각(rad) 폴백 테이블.
# 평소에는 tf_static에서 조회하고, 조회가 실패했을 때만 쓴다.
IR_SENSOR_YAW_FALLBACK = {
    'ir_intensity_side_left': math.radians(75.0),
    'ir_intensity_left': math.radians(50.0),
    'ir_intensity_front_left': math.radians(25.0),
    'ir_intensity_front_center_left': math.radians(0.0),
    'ir_intensity_front_center_right': math.radians(-25.0),
    'ir_intensity_front_right': math.radians(-50.0),
    'ir_intensity_right': math.radians(-75.0),
}


def decode_compressed_depth(data):
    """Decode ROS compressed_depth_image_transport payload: 12-byte header
    (depthQuantA/depthQuantB float32 params) followed by PNG-compressed 16UC1 data.

    이 로봇의 OAK-D 스트림은 헤더가 (format=0, quantA=0, quantB=0)로 양자화 미적용이라
    12바이트를 건너뛰고 PNG를 그대로 디코드하면 mm 단위 16UC1이 나온다.
    """
    if len(data) <= 12:
        return None
    buf = np.frombuffer(bytes(data[12:]), dtype=np.uint8)
    return cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)


def sample_depth_patch(depth_img, u, v, patch_size, depth_scale):
    """Median of valid (nonzero) depth values in a patch_size x patch_size window
    centered at pixel (u, v), converted to meters via depth_scale. Returns None if
    no valid pixels are found."""
    h, w = depth_img.shape[:2]
    u, v = int(round(u)), int(round(v))
    half = patch_size // 2
    u0, u1 = max(0, u - half), min(w, u + half + 1)
    v0, v1 = max(0, v - half), min(h, v + half + 1)
    if u0 >= u1 or v0 >= v1:
        return None
    region = depth_img[v0:v1, u0:u1].astype(np.float32)
    valid = region[region > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid)) * depth_scale


def estimate_person_depth(depth_img, bbox_xyxy, depth_scale, percentile=20.0,
                          min_valid_pixels=20, z_min=0.3, z_max=6.0):
    """bbox 안쪽에서 '사람 표면'을 대표하는 깊이(m)를 로버스트하게 추정한다. 실패 시 None.

    사람은 자기 bbox 안에서 배경보다 항상 **더 가깝다**는 성질을 이용해 낮은 분위수를 쓴다
    (평균/중앙값은 배경이 넓게 잡히면 배경 쪽으로 끌려간다). 실루엣 경계에서 사람과 배경이
    섞이는 걸 줄이려고 좌우 20%를 잘라낸 안쪽만 본다.

    용도는 발끝 depth 검증이다 - 발끝 픽셀 주변이 통째로 배경을 찍는 경우가 실측으로 확인됐고
    (my_new_bag5: 발끝 depth로 배경 거리 2.51m가 반복 등장), 그때 이 값으로 대체하면 원시
    측정 점프가 0.5m 초과 12회 -> 4회로 줄었다.
    """
    if depth_img is None:
        return None
    h, w = depth_img.shape[:2]
    x1, y1, x2, y2 = (int(round(c)) for c in bbox_xyxy)
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w, x2), min(h, y2)
    if x2 <= x1 or y2 <= y1:
        return None
    margin = int((x2 - x1) * 0.2)
    x1, x2 = x1 + margin, x2 - margin
    if x2 <= x1:
        return None
    region = depth_img[y1:y2, x1:x2].astype(np.float32)
    valid = region[region > 0] * depth_scale
    valid = valid[(valid > z_min) & (valid < z_max)]
    if valid.size < min_valid_pixels:
        return None
    return float(np.percentile(valid, percentile))


def backproject_pixel(u, v, z, fx, fy, cx, cy):
    """핀홀 역투영: 픽셀 (u, v) + depth z(m) -> 카메라 optical frame 3D 좌표(m).

    optical frame 규약이라 x=오른쪽, y=아래, z=정면이다. tf2가 이후 base_link/map으로 바꾼다.
    """
    return (
        (float(u) - cx) * z / fx,
        (float(v) - cy) * z / fy,
        float(z),
    )


def estimate_foot_pixel(kp_xy, kp_conf, bbox_xyxy, kp_conf_threshold):
    """사람 한 명의 keypoint에서 지면 접지점(발끝) 픽셀을 뽑고 산출 방식 등급을 매긴다.

    문서 4번의 "발끝이 가려져 직접 검출되지 않는 경우"를 3등급으로 구분한다:
      toe_direct     - ankle keypoint를 직접 썼다 (가장 신뢰)
      knee_corrected - ankle이 없어 무릎 x + bbox 하단 y로 보정했다
      angle_only     - keypoint를 못 써서 bbox 하단 중앙으로 추정했다

    kp_xy: (17, 2) 픽셀좌표, kp_conf: (17,) 신뢰도, bbox_xyxy: (x1, y1, x2, y2)
    반환: (u, v, grade)
    """
    x1, y1, x2, y2 = bbox_xyxy

    def valid(idx):
        return (
            kp_xy is not None
            and kp_conf is not None
            and idx < len(kp_conf)
            and float(kp_conf[idx]) >= kp_conf_threshold
        )

    ankles = [i for i in (KP_LEFT_ANKLE, KP_RIGHT_ANKLE) if valid(i)]
    if ankles:
        u = sum(float(kp_xy[i][0]) for i in ankles) / len(ankles)
        v = sum(float(kp_xy[i][1]) for i in ankles) / len(ankles)
        return u, v, 'toe_direct'

    knees = [i for i in (KP_LEFT_KNEE, KP_RIGHT_KNEE) if valid(i)]
    if knees:
        # 무릎은 보이는데 발끝이 안 보이는 상황 - 좌우 위치는 무릎을 믿고
        # 지면 접지 높이는 bbox 하단으로 대신한다.
        u = sum(float(kp_xy[i][0]) for i in knees) / len(knees)
        return u, float(y2), 'knee_corrected'

    return (float(x1) + float(x2)) / 2.0, float(y2), 'angle_only'


def truncation_ratio(bbox_xyxy, top_margin_px):
    """bbox 상단이 이미지 위쪽 경계에 붙었는지(=상반신이 잘렸는지) 0~1로 반환한다.

    이 카메라는 지면 0.244m에 수평으로 달려 있어 2.5m 이내에서는 머리가 항상 잘린다.
    하위 노드(재식별/트래킹)가 "이 검출은 몸 일부만 본 것"임을 알고 라이다로 주도권을
    넘길지 판단할 수 있도록 신호로 내보낸다. 0이면 잘리지 않은 것.
    """
    y1 = float(bbox_xyxy[1])
    y2 = float(bbox_xyxy[3])
    if y1 > top_margin_px:
        return 0.0
    height = max(y2 - y1, 1.0)
    # 경계에 얼마나 깊이 물렸는지를 bbox 높이로 정규화 (경계 접촉=0에 가깝고 더 물릴수록 증가)
    return float(min(1.0, (top_margin_px - y1 + 1.0) / height))


def yaw_from_quaternion(q):
    """geometry_msgs Quaternion -> yaw(rad)."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angular_diff(a, b):
    """두 각도의 최소 차이 절댓값(rad)."""
    d = (a - b + math.pi) % (2.0 * math.pi) - math.pi
    return abs(d)
