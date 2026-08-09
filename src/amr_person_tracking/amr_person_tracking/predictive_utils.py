#!/usr/bin/env python3
"""
predictive_utils.py

predictive_avoidance_node이 쓰는 순수 로직. rclpy에 의존하지 않아 노드 없이 단위 검증이 가능하다.

reid_tracking_node의 tracking_utils.Track은 지수평활로 단순화한 것이었지만, 이 노드는 문서가
"6번 예측적 회피 = 칼만필터로 속도 추정+예측"이라고 이 파이프라인의 실제 칼만필터 담당으로
지정한 노드라 여기서는 표준적인 등속도 모델(constant-velocity) 2D 칼만필터를 그대로 쓴다.
"""

import math

import numpy as np


class ConstantVelocityKalman2D:
    """상태 [x, y, vx, vy]의 등속도 모델 칼만필터.

    Δt는 항상 호출자가 넘긴 stamp(메시지 header.stamp, 즉 촬영/스캔 시각)로 계산한다 -
    수신 시각을 쓰면 로봇<->노트북 네트워크 지연으로 프레임 간격이 불규칙해져 속도 추정이 튄다.
    """

    def __init__(self, x, y, stamp, process_noise=1.0, measurement_noise=0.1,
                 initial_velocity_variance=4.0):
        self.state = np.array([x, y, 0.0, 0.0], dtype=float)
        self.P = np.diag([
            measurement_noise, measurement_noise,
            initial_velocity_variance, initial_velocity_variance,
        ]).astype(float)
        self.last_stamp = stamp
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise

    def _predict(self, stamp):
        dt = stamp - self.last_stamp
        if dt <= 0:
            return
        F = np.array([
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
        q = self.process_noise
        # 등가속도 백색잡음(piecewise white noise) 모델의 표준 이산화 공정잡음
        Q = q * np.array([
            [dt ** 4 / 4, 0.0, dt ** 3 / 2, 0.0],
            [0.0, dt ** 4 / 4, 0.0, dt ** 3 / 2],
            [dt ** 3 / 2, 0.0, dt ** 2, 0.0],
            [0.0, dt ** 3 / 2, 0.0, dt ** 2],
        ])
        self.state = F @ self.state
        self.P = F @ self.P @ F.T + Q
        self.last_stamp = stamp

    def _predicted(self, stamp):
        """상태를 **바꾸지 않고** stamp 시점의 (state, P)를 돌려준다.

        게이팅 판정은 "이 관측을 받아들일지" 정하기 전에 하므로 필터 상태를 진행시키면 안 된다
        (받아들이지 않을 관측 때문에 공분산이 커지면 다음 판정까지 흔들린다).
        """
        dt = stamp - self.last_stamp
        if dt <= 0:
            return self.state.copy(), self.P.copy()
        F = np.array([
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
        q = self.process_noise
        Q = q * np.array([
            [dt ** 4 / 4, 0.0, dt ** 3 / 2, 0.0],
            [0.0, dt ** 4 / 4, 0.0, dt ** 3 / 2],
            [dt ** 3 / 2, 0.0, dt ** 2, 0.0],
            [0.0, dt ** 3 / 2, 0.0, dt ** 2],
        ])
        return F @ self.state, F @ self.P @ F.T + Q

    def predicted_position(self, stamp):
        """필터 상태를 바꾸지 않고 stamp 시점의 예측 위치."""
        state, _P = self._predicted(stamp)
        return float(state[0]), float(state[1])

    def mahalanobis(self, x, y, stamp, measurement_noise=None):
        """관측 (x, y)의 마할라노비스 거리. 게이팅용이라 필터 상태를 바꾸지 않는다.

        유클리드 고정반경 게이트와 달리 **불확실도로 정규화**한 거리라, 오래 못 본 트랙(공분산이
        큼)에는 자연히 관대해지고 방금 갱신된 트랙에는 엄격해진다. 2자유도 카이제곱이므로
        임계 5.991이 95%, 9.210이 99% 신뢰구간에 해당한다(DeepSORT가 쓰는 방식).
        """
        state, P = self._predicted(stamp)
        H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        r = self.measurement_noise if measurement_noise is None else measurement_noise
        S = H @ P @ H.T + np.diag([r, r])
        residual = np.array([x, y]) - H @ state
        try:
            return float(math.sqrt(residual @ np.linalg.inv(S) @ residual))
        except np.linalg.LinAlgError:
            return float('inf')

    def update(self, x, y, stamp, measurement_noise=None):
        """새 관측치로 predict-then-correct를 수행한다. stamp가 과거면(순서 뒤바뀐 메시지) 무시.

        measurement_noise를 주면 그 관측에 한해 R을 덮어쓴다 - 검출기가 자기 불확실도를
        covariance로 신고하므로(oakd_detector_node의 GRADE_SIGMA + 동기 시간차), 프레임마다
        다른 신뢰도를 그대로 반영할 수 있다.
        """
        if stamp < self.last_stamp:
            return
        self._predict(stamp)
        H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        r = self.measurement_noise if measurement_noise is None else measurement_noise
        R = np.diag([r, r])
        z = np.array([x, y])
        residual = z - H @ self.state
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ residual
        self.P = (np.eye(4) - K @ H) @ self.P

    def position(self):
        return float(self.state[0]), float(self.state[1])

    def velocity(self):
        return float(self.state[2]), float(self.state[3])

    def speed(self):
        return float(math.hypot(self.state[2], self.state[3]))

    def predict_future(self, dt):
        """현재 상태에서 dt초 후 위치를 등속도로 예측한다 (필터 내부 상태는 바꾸지 않음)."""
        return float(self.state[0] + self.state[2] * dt), float(self.state[1] + self.state[3] * dt)


def approach_speed(robot_x, robot_y, track_x, track_y, vx, vy):
    """트랙 속도 중 로봇 쪽으로 다가오는 성분(m/s). 양수=접근 중, 0 이하=정지/이탈."""
    dx = robot_x - track_x
    dy = robot_y - track_y
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return math.hypot(vx, vy)
    ux, uy = dx / dist, dy / dist  # 트랙 -> 로봇 단위벡터
    return vx * ux + vy * uy


class LegKalmanTracker:
    """짝짓기(pair_legs) 이전의 개별 라이다 다리 후보를 프레임 간에 매칭해 등속도 칼만필터로
    속도를 추정한다 - leg_detector_bridge_node가 StaticBackgroundFilter에 "이 위치를 정적
    배경으로 확정해도 되는지" 판단하는 데 쓴다 (leg_detection_utils.StaticBackgroundFilter의
    설계 변경 이력 참고).

    이전 두 설계(사람/다리쌍 단위 정지 판정, 그리드 셀 연속관측)가 모두 실패한 이유는 공통적으로
    "관측이 한 번이라도 끊기거나(occlusion) 짝짓기 상대가 바뀌면 판정이 흐트러진다"는 것이었다.
    칼만필터는 관측 사이 간격(dt)이 커져도(사람이 잠깐 가려서 몇 스캔 건너뜀) 그 구간만큼
    공정잡음(Q)이 커질 뿐 상태(위치/속도) 자체가 리셋되지 않는다 - 다시 보였을 때 위치가
    그대로면 속도 추정도 그대로 0에 가깝게 유지된다. 그래서 "처음 본 뒤 누적 나이 >=
    confirm_duration_sec 이고 지금 추정 속도 <= stationary_speed_threshold"를 정지 확정
    기준으로 쓰면, 사람이 물체 앞을 반복해서 오가며 매번 공백을 만들어도(실측으로 확인된 실패
    사례) 결국 확정된다 - 그리드 셀 방식처럼 공백마다 타이머가 리셋되지 않기 때문이다.

    stationary_speed_threshold 기본값(0.01m/s)은 이전 세션에서 실측한 "서 있는 사람"의 위치
    드리프트(~7초에 ~10.6cm, 약 0.015m/s)보다 낮게 잡아, 실제 사람은 이 기준을 만족하지
    못하게 여유를 뒀다.
    """

    def __init__(self, match_gate=0.10, confirm_duration_sec=3.0,
                 stationary_speed_threshold=0.01, kf_timeout_sec=5.0,
                 process_noise=0.01, measurement_noise=0.0025):
        self.match_gate = match_gate
        self.confirm_duration_sec = confirm_duration_sec
        self.stationary_speed_threshold = stationary_speed_threshold
        self.kf_timeout_sec = kf_timeout_sec
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        self._kalman = {}   # leg_id -> ConstantVelocityKalman2D
        self._created = {}  # leg_id -> 최초 관측 시각
        self._next_id = 1

    def update(self, leg_positions, stamp):
        """이번 스캔의 개별 다리 후보 (x, y) 리스트를 매칭/갱신하고, 지금 막 "정지 확정" 조건을
        만족한 위치들의 목록을 반환한다 (호출자가 StaticBackgroundFilter.confirm_static()에
        넘기면 됨 - 이미 확정된 위치를 다시 반환해도 confirm_static()은 멱등이라 안전하다)."""
        matched_ids = self._match(leg_positions)
        stationary = []
        for (x, y), leg_id in zip(leg_positions, matched_ids):
            if leg_id is None:
                leg_id = self._next_id
                self._next_id += 1
                self._kalman[leg_id] = ConstantVelocityKalman2D(
                    x, y, stamp,
                    process_noise=self.process_noise,
                    measurement_noise=self.measurement_noise)
                self._created[leg_id] = stamp
            else:
                self._kalman[leg_id].update(x, y, stamp)

            kf = self._kalman[leg_id]
            age = stamp - self._created[leg_id]
            if age >= self.confirm_duration_sec and kf.speed() <= self.stationary_speed_threshold:
                stationary.append(kf.position())

        self._prune(stamp)
        return stationary

    def _match(self, leg_positions):
        """이번 스캔의 다리 후보들을 기존 칼만필터에 배타적(1:1)으로, 가까운 것부터 그리디로
        배정한다. tracking_utils.assign_tracks와 같은 패턴이지만 Track이 아니라
        ConstantVelocityKalman2D를 다루므로 여기 별도로 둔다."""
        n = len(leg_positions)
        results = [None] * n
        claimed = set()
        pending = set(range(n))

        while pending:
            best = None  # (dist, idx, leg_id)
            for idx in pending:
                x, y = leg_positions[idx]
                best_id, best_dist = None, None
                for lid, kf in self._kalman.items():
                    if lid in claimed:
                        continue
                    kx, ky = kf.position()
                    d = math.hypot(x - kx, y - ky)
                    if d <= self.match_gate and (best_dist is None or d < best_dist):
                        best_id, best_dist = lid, d
                if best_id is not None and (best is None or best_dist < best[0]):
                    best = (best_dist, idx, best_id)
            if best is None:
                break
            _, idx, lid = best
            results[idx] = lid
            claimed.add(lid)
            pending.discard(idx)

        return results

    def _prune(self, stamp):
        stale = [lid for lid, kf in self._kalman.items() if stamp - kf.last_stamp > self.kf_timeout_sec]
        for lid in stale:
            del self._kalman[lid]
            del self._created[lid]


def ring_points(cx, cy, z, radius, count):
    """cx, cy를 중심으로 반경 radius인 원 둘레에 count개 점 + 중심점 1개를 반환한다."""
    points = [(cx, cy, z)]
    for i in range(count):
        theta = 2.0 * math.pi * i / count
        points.append((cx + radius * math.cos(theta), cy + radius * math.sin(theta), z))
    return points
