#!/usr/bin/env python3
"""
tracking_utils.py

reid_tracking_node이 쓰는 순수 트래킹 로직. rclpy에 의존하지 않아 노드 없이 단위 검증이 가능하다.

[의도적 단순화]
문서(vision_pipeline.md)는 "칼만필터로 좌표 블렌딩"을 말하지만, 여기서는 완전한 2D 칼만필터
행렬 대신 지수평활(exponential smoothing) 기반 등속도 모델을 쓴다. predictive_avoidance_node가
이 파이프라인에서 실제 칼만필터를 맡는 노드로 문서에 이미 지정돼 있어(문서 6번), 이 노드 안의
트랙 상태 유지는 그보다 가벼운 방식으로도 "출처 전환 시 좌표가 튀는 것을 완화"라는 목적은
충분히 달성한다.
"""

import math

# 서로 다른 센서가 "같은 순간"을 보고할 때의 클럭 지터 허용치(초). 이보다 더 과거인 관측만
# 시간 역순으로 취급해 거부한다 (Track.update 참고).
_STALE_EPS = 0.01


class Track:
    """트랙 하나의 상태. 위치/속도는 map 좌표계(m) 기준."""

    def __init__(self, track_id, x, y, stamp, source):
        self.id = track_id
        self.x = x
        self.y = y
        self.vx = 0.0
        self.vy = 0.0
        self.last_stamp = stamp
        self.last_source = source
        self.created_stamp = stamp

    def predict(self, stamp, max_dt=None):
        """등속도 모델로 stamp 시점의 위치를 외삽한다.

        max_dt를 주면 그보다 오래된 갱신은 그만큼만 외삽한다 - 트랙이 한동안 갱신되지 않은 채
        (예: 라이다 락온 후보를 여러 프레임 모으는 동안) 오래된 속도로 무한정 미래를 예측하면
        아주 살짝의 속도 오차도 시간이 지날수록 걷잡을 수 없이 커진다.
        """
        dt = stamp - self.last_stamp
        if max_dt is not None:
            dt = max(-max_dt, min(dt, max_dt))
        return self.x + self.vx * dt, self.y + self.vy * dt

    def update(self, x, y, stamp, source, velocity_alpha=0.5, position_alpha=1.0, max_speed=None):
        """새 관측으로 트랙을 갱신한다.

        velocity_alpha: 속도 추정의 지수평활 계수 (1.0=관측만 신뢰, 0=이전 속도 유지)
        position_alpha: 위치 갱신 블렌딩 계수 (1.0=관측 위치로 바로 스냅, <1.0이면 이전 위치와
            섞어 완화한다 - 출처가 바뀌는 순간에만 낮춰 쓴다)
        max_speed: 유한차분으로 구한 순간 속도의 최대 크기(m/s). position_alpha<1.0인 블렌딩
            업데이트 직후 반대 방향으로 스냅하는 것처럼, 아주 짧은 dt 사이에 위치가 오락가락하면
            (x-self.x)/dt가 사람이 낼 수 없는 속도로 튀어 이후 예측을 크게 틀어지게 만들 수 있다
            - 그런 관측은 속도 추정에서 클램프해 무시한다.

        시간 역순(다른 출처가 더 늦게 도착하는 등)으로 들어온 관측이 상태를 과거로 되감지
        않도록, stamp가 last_stamp보다 유의미하게 과거면 갱신을 통째로 무시한다. 반환값으로
        실제 반영 여부를 알려주지만, 현재 모든 호출부는 부수효과만 쓰고 반환값을 버린다.
        """
        dt = stamp - self.last_stamp
        if dt < -_STALE_EPS:
            return False
        dt = max(dt, 0.0)
        if dt > 1e-6:
            vx_meas = (x - self.x) / dt
            vy_meas = (y - self.y) / dt
            if max_speed is not None:
                speed = math.hypot(vx_meas, vy_meas)
                if speed > max_speed:
                    scale = max_speed / speed
                    vx_meas *= scale
                    vy_meas *= scale
            self.vx = velocity_alpha * vx_meas + (1.0 - velocity_alpha) * self.vx
            self.vy = velocity_alpha * vy_meas + (1.0 - velocity_alpha) * self.vy
        self.x = self.x + position_alpha * (x - self.x)
        self.y = self.y + position_alpha * (y - self.y)
        self.last_stamp = max(self.last_stamp, stamp)
        self.last_source = source
        return True


def distance(x0, y0, x1, y1):
    return math.hypot(x1 - x0, y1 - y0)


def gate_radius(dt, gating_max_speed, min_gate):
    """dt초 동안 gating_max_speed(m/s)로 이동 가능한 최대 거리. dt가 아주 작을 때(같은 순간에
    가까운 두 출처가 들어오는 경우 등) 게이트가 0에 가까워지지 않도록 최소값을 보장한다."""
    return max(min_gate, gating_max_speed * abs(dt))


def match_track(tracks, x, y, stamp, gating_max_speed, min_gate, exclude_ids=None):
    """기존 트랙 중 (x, y)에 가장 가깝고 게이트 이내인 트랙 id를 찾는다. 없으면 None.

    exclude_ids: 이미 같은 콜백에서 다른 검출에 배정된 트랙 id 집합. 한 콜백 안에서 서로 다른
    검출이 같은 트랙을 중복으로 차지하지 않도록 assign_tracks()가 채워 넘긴다.
    """
    exclude_ids = exclude_ids or ()
    best_id, best_dist = None, None
    for tid, tr in tracks.items():
        if tid in exclude_ids:
            continue
        px, py = tr.predict(stamp)
        d = distance(px, py, x, y)
        gate = gate_radius(stamp - tr.last_stamp, gating_max_speed, min_gate)
        if d <= gate and (best_dist is None or d < best_dist):
            best_id, best_dist = tid, d
    return best_id


def assign_tracks(tracks, detections, stamp, gating_max_speed, min_gate):
    """한 콜백에 들어온 여러 검출을 기존 트랙에 배타적(1:1)으로 배정한다.

    detections: (x, y) 튜플의 리스트. 반환값은 detections와 같은 길이의 리스트로, 각 원소는
    매칭된 트랙 id 또는 게이트 안에 트랙이 없으면 None이다.

    Hungarian 알고리즘 같은 전역 최적 할당은 아니고, 아직 배정 안 된 검출들 중 최근접 거리가
    가장 작은 것부터 그리디로 확정해 배정된 트랙을 제외해가는 방식이다. 한 콜백의 검출 수가
    사람 수 수준(수 개)이라 O(n^2)로 충분하다.
    """
    n = len(detections)
    results = [None] * n
    claimed = set()
    pending = set(range(n))

    while pending:
        best = None  # (dist, idx, track_id)
        for idx in pending:
            x, y = detections[idx]
            tid = match_track(tracks, x, y, stamp, gating_max_speed, min_gate, exclude_ids=claimed)
            if tid is None:
                continue
            px, py = tracks[tid].predict(stamp)
            d = distance(px, py, x, y)
            if best is None or d < best[0]:
                best = (d, idx, tid)
        if best is None:
            break
        _, idx, tid = best
        results[idx] = tid
        claimed.add(tid)
        pending.discard(idx)

    return results


def extract_person_position(detection):
    """Detection3D에서 class_id=='person' 가설의 위치 (x, y, z)를 뽑는다. 없으면 None.

    oakd_detector_node/leg_detector_bridge_node 둘 다 이 관례로 'person' 가설을 채운다.
    """
    for result in detection.results:
        if result.hypothesis.class_id == 'person':
            p = result.pose.pose.position
            return p.x, p.y, p.z
    return None


def velocity_similarity(vx0, vy0, vx1, vy1):
    """두 속도 벡터의 유사도 (방향의 코사인 유사도 x 크기 비율, 0~1). 둘 다 정지에 가까우면 1.0."""
    s0 = math.hypot(vx0, vy0)
    s1 = math.hypot(vx1, vy1)
    if s0 < 1e-3 and s1 < 1e-3:
        return 1.0
    if s0 < 1e-3 or s1 < 1e-3:
        return 0.0
    cos_sim = (vx0 * vx1 + vy0 * vy1) / (s0 * s1)
    speed_ratio = min(s0, s1) / max(s0, s1)
    return max(0.0, cos_sim) * speed_ratio


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9
