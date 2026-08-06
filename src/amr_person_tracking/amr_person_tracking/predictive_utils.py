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

    def update(self, x, y, stamp):
        """새 관측치로 predict-then-correct를 수행한다. stamp가 과거면(순서 뒤바뀐 메시지) 무시."""
        if stamp < self.last_stamp:
            return
        self._predict(stamp)
        H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        R = np.diag([self.measurement_noise, self.measurement_noise])
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


def ring_points(cx, cy, z, radius, count):
    """cx, cy를 중심으로 반경 radius인 원 둘레에 count개 점 + 중심점 1개를 반환한다."""
    points = [(cx, cy, z)]
    for i in range(count):
        theta = 2.0 * math.pi * i / count
        points.append((cx + radius * math.cos(theta), cy + radius * math.sin(theta), z))
    return points
