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

from scipy.optimize import linear_sum_assignment

# 서로 다른 센서가 "같은 순간"을 보고할 때의 클럭 지터 허용치(초). 이보다 더 과거인 관측만
# 시간 역순으로 취급해 거부한다 (Track.update 참고).
_STALE_EPS = 0.01

# assign_tracks에서 게이트 밖(허용 불가) 쌍에 매기는 비용. linear_sum_assignment는 모든
# 칸에 값이 있어야 하므로, 실제 나올 수 있는 어떤 거리보다도 훨씬 큰 값으로 채워 "이 쌍은
# 배정되지 않는 한 다른 선택지가 없을 때만 최후의 수단으로 고려된다"를 표현한다. 배정 후
# 이 값 이상인 결과는 게이트 밖으로 간주해 버린다(assign_tracks 참고).
_UNREACHABLE_COST = 1e6


class Track:
    """트랙 하나의 상태. 위치/속도는 map 좌표계(m) 기준."""

    def __init__(self, track_id, x, y, stamp, source, kalman=None):
        self.id = track_id
        self.x = x
        self.y = y
        self.vx = 0.0
        self.vy = 0.0
        self.last_stamp = stamp
        self.last_source = source
        self.created_stamp = stamp
        # 등속도 칼만필터(predictive_utils.ConstantVelocityKalman2D). 주어지면 위치/속도를
        # 이걸로 추정하고, 없으면 기존 지수평활 경로를 그대로 쓴다.
        # [왜 필요한가] 예전에는 position_alpha가 평소 1.0이라 트랙 위치를 관측값에 그대로
        # 스냅했다. 그래서 검출 노이즈가 1:1로 추종 좌표 점프가 됐다(실측: 원시 측정 0.5m 초과
        # 12회 = 최종 출력 0.5m 초과 12회로 정확히 일치). 칼만필터는 관측 불확실도(R)를 감안해
        # 섞으므로 스파이크가 그대로 출력되지 않는다.
        self.kalman = kalman
        # 외형(ReID) 임베딩. 임베딩을 주는 출처가 없으면 계속 None으로 남고, 그 경우
        # 매칭은 기존과 동일하게 위치 기반으로만 동작한다.
        self.embedding = None

    def update_embedding(self, embedding, alpha=0.9):
        """외형 임베딩을 EMA로 갱신한다.

        한 프레임의 조명/자세/모션블러가 트랙의 외형을 통째로 덮어쓰지 않도록 평활한다.
        alpha는 "기존 값을 얼마나 유지할지"로, 1.0에 가까울수록 과거를 더 신뢰한다.
        코사인 유사도로 비교하므로 갱신 후 다시 정규화한다.
        """
        if embedding is None:
            return
        if self.embedding is None:
            self.embedding = list(embedding)
            return
        blended = [alpha * old + (1.0 - alpha) * new
                   for old, new in zip(self.embedding, embedding)]
        norm = math.sqrt(sum(v * v for v in blended))
        self.embedding = [v / norm for v in blended] if norm > 0.0 else blended

    def predict(self, stamp, max_dt=None):
        """등속도 모델로 stamp 시점의 위치를 외삽한다.

        max_dt를 주면 그보다 오래된 갱신은 그만큼만 외삽한다 - 트랙이 한동안 갱신되지 않은 채
        (예: 라이다 락온 후보를 여러 프레임 모으는 동안) 오래된 속도로 무한정 미래를 예측하면
        아주 살짝의 속도 오차도 시간이 지날수록 걷잡을 수 없이 커진다.
        """
        if self.kalman is not None and max_dt is None:
            return self.kalman.predicted_position(stamp)
        dt = stamp - self.last_stamp
        if max_dt is not None:
            dt = max(-max_dt, min(dt, max_dt))
        if self.kalman is not None:
            # max_dt로 외삽을 제한해야 하는 호출부(라이다 후보 게이팅)는 칼만 상태에서
            # 속도만 가져와 직접 제한 외삽한다 - 오래된 속도로 무한정 미래를 예측하지 않기 위해.
            kx, ky = self.kalman.position()
            kvx, kvy = self.kalman.velocity()
            return kx + kvx * dt, ky + kvy * dt
        return self.x + self.vx * dt, self.y + self.vy * dt

    def mahalanobis(self, x, y, stamp, measurement_noise=None):
        """관측까지의 마할라노비스 거리. 칼만필터가 없으면 None(호출부가 유클리드로 폴백)."""
        if self.kalman is None:
            return None
        return self.kalman.mahalanobis(x, y, stamp, measurement_noise)

    def update(self, x, y, stamp, source, velocity_alpha=0.5, position_alpha=1.0, max_speed=None,
               measurement_noise=None):
        """새 관측으로 트랙을 갱신한다.

        칼만필터가 붙어 있으면 그쪽으로 갱신하고 x/y/vx/vy를 필터 추정치로 동기화한다.
        이때 velocity_alpha/position_alpha/max_speed는 무시된다 - 지수평활 시절의 노브라
        필터가 관측 불확실도(measurement_noise)로 같은 역할을 더 원칙적으로 수행한다.

        measurement_noise: 이 관측의 분산(m^2). 검출기가 covariance로 신고한 값을 그대로
            넣으면 프레임별 신뢰도가 추정에 반영된다.

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

        if self.kalman is not None:
            self.kalman.update(x, y, stamp, measurement_noise=measurement_noise)
            self.x, self.y = self.kalman.position()
            self.vx, self.vy = self.kalman.velocity()
            self.last_stamp = max(self.last_stamp, stamp)
            self.last_source = source
            return True

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


def cosine_similarity(a, b):
    """두 외형 임베딩의 코사인 유사도. 둘 중 하나라도 없으면 None(=비교 불가).

    발행 쪽(oakd_detector_node)이 이미 정규화해 보내지만, 트랙 쪽 EMA 갱신 결과나 외부
    입력이 정규화돼 있지 않을 수 있어 여기서 다시 나눠준다.
    """
    if a is None or b is None or len(a) != len(b):
        return None
    dot = sum(p * q for p, q in zip(a, b))
    na = math.sqrt(sum(p * p for p in a))
    nb = math.sqrt(sum(q * q for q in b))
    if na <= 0.0 or nb <= 0.0:
        return None
    return dot / (na * nb)


def resolve_call_designation(tracks, call, now, radius, ttl):
    """웹캠 호출 좌표로 추종 대상을 정한다. 순수 함수라 단위테스트로 고정한다.

    반환: ('designate', track_id) | ('wait', None) | ('expire', None)

    호출 좌표는 "이 사람을 따라와라"는 사람의 명시적 지정이다. 로봇이 그 좌표까지
    이동하는 데 수 초가 걸리므로, 도착 시점에 그 근처에서 가장 가까운 트랙을 고른다.

    후보가 없으면 'designate'가 아니라 **'wait'**를 돌려주는 것이 핵심이다. 호출자가
    아직 카메라에 안 잡혔는데 기존 정책(가장 먼저 생긴 트랙)으로 폴백하면, 이동하는
    동안 지나가던 사람을 붙잡고 도착해서는 엉뚱한 사람을 따라가게 된다.

    radius <= 0 이면 거리 제한 없음(가장 가까운 트랙을 무조건 채택), ttl <= 0 이면
    만료 없음 - 이 패키지의 다른 파라미터와 같은 규약이다. 다만 둘 다 끄는 것은
    권장하지 않는다: 만료가 없으면 오래된 호출이 영원히 재선정을 막는다.
    """
    if call is None:
        return ('wait', None)
    cx, cy, call_stamp = call
    if ttl > 0.0 and now - call_stamp > ttl:
        return ('expire', None)
    best_id, best_d = None, None
    for tid, tr in tracks.items():
        d = distance(tr.x, tr.y, cx, cy)
        if radius > 0.0 and d > radius:
            continue
        if best_d is None or d < best_d:
            best_id, best_d = tid, d
    if best_id is None:
        return ('wait', None)
    return ('designate', best_id)


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


def assign_tracks(tracks, detections, stamp, gating_max_speed, min_gate,
                  embeddings=None, appearance_weight=0.0, appearance_reject=None,
                  mahalanobis_gate=None, measurement_noises=None):
    """한 콜백에 들어온 여러 검출을 기존 트랙에 배타적(1:1)으로, 전체 비용 합이 최소가 되도록
    배정한다(Hungarian/linear_sum_assignment, scipy.optimize).

    detections: (x, y) 튜플의 리스트. 반환값은 detections와 같은 길이의 리스트로, 각 원소는
    매칭된 트랙 id 또는 게이트 안에 트랙이 없으면 None이다.

    이전에는 아직 배정 안 된 검출들 중 최근접 거리가 가장 작은 것부터 그리디로 확정해가는
    방식이었다. 실측(my_new_bag3 2인 교차 구간)에서 이 그리디 방식이 지역적으로는 매 순간
    "가장 가까운" 선택을 하지만 전체적으로 봤을 때 더 나쁜 조합(예: A는 살짝 먼 자기 짝을
    두고 B의 짝을 가로채, 결과적으로 A-B 모두에게 나쁜 배정)을 만들 수 있음을 확인해 전역
    최적 배정으로 교체했다. 게이트 밖(허용 불가) 쌍은 매우 큰 비용(_UNREACHABLE_COST)으로
    채워 사실상 배제하되, 다른 선택지가 전혀 없을 때만 최후 수단으로 고려되게 한다 - 배정
    후에는 그 비용 이상인 결과를 게이트 밖으로 간주해 버린다.

    embeddings: detections와 같은 길이의 외형 임베딩 리스트(없는 항목은 None). 주어지면
        비용에 `appearance_weight * (1 - 코사인유사도)`를 더한다. 게이트 판정 자체는 계속
        위치로만 하고(외형이 비슷하다는 이유로 멀리 있는 트랙에 붙는 것을 막는다), 외형은
        "게이트를 통과한 후보들 중 어느 것을 고를지"에만 개입한다 - DeepSORT가 motion
        gating 위에 appearance cost를 얹는 것과 같은 구조다. 둘 중 한쪽이라도 임베딩이
        없으면 그 쌍은 외형 항 없이 거리만으로 비교돼 기존 동작으로 degrade한다.

    mahalanobis_gate: 주어지면(예: 2자유도 카이제곱 95% = 5.991의 제곱근 ≈ 2.448) 유클리드
        고정반경 대신 **불확실도로 정규화한 거리**로 게이팅한다. 오래 못 본 트랙은 공분산이
        커져 자연히 관대해지고 방금 갱신된 트랙은 엄격해진다(DeepSORT 방식). 칼만필터가
        없는 트랙은 계산이 불가능하므로 기존 유클리드 게이트로 폴백한다.
    measurement_noises: detections와 같은 길이의 관측 분산(m^2) 리스트. 검출기가 신고한
        covariance를 그대로 넘기면 프레임별 신뢰도가 게이팅에 반영된다.
    """
    n = len(detections)
    results = [None] * n
    if n == 0 or not tracks:
        return results

    track_ids = list(tracks.keys())
    cost = [[_UNREACHABLE_COST] * len(track_ids) for _ in range(n)]
    for i, (x, y) in enumerate(detections):
        det_emb = embeddings[i] if embeddings is not None and i < len(embeddings) else None
        r_i = (measurement_noises[i]
               if measurement_noises is not None and i < len(measurement_noises) else None)
        for j, tid in enumerate(track_ids):
            tr = tracks[tid]
            px, py = tr.predict(stamp)
            # 게이팅 거리는 예측 위치와 **마지막 관측 위치 중 가까운 쪽**을 쓴다.
            #
            # 예측만 쓰면, 속도 추정이 틀렸을 때 예측이 매칭을 돕는 대신 방해한다. 실기
            # (evidence/live_run_0809_1658.log)에서 생성 0.4초짜리 추종 트랙이 이렇게 죽었다:
            #   생성 track 2 ... 최근접 track 1 저장 0.27m / 예측 0.35m (v=0.87m/s)
            # 사람은 사실상 제자리인데(실이동 0.27m) 발끝 depth 지터에 지수평활을 먹인 속도가
            # 0.87m/s로 잡혀 예측을 게이트(0.3m) 밖으로 밀어냈다. 검출은 그 프레임에 하나뿐이라
            # 경쟁도 없었는데 새 트랙이 생기고, 추종 트랙은 0.30초 만에 굶어 죽었다.
            #
            # 둘 중 가까운 쪽을 쓰면 예측은 recall을 높이는 쪽으로만 작용한다. 게이트 반경
            # 자체는 그대로라 서로 다른 사람을 섞을 위험은 늘지 않는다(반경 밖은 여전히 거부).
            # 비용에는 예측 거리를 그대로 써서, 통과한 후보들 사이의 우선순위는 바꾸지 않는다.
            d = distance(px, py, x, y)
            gate_d = min(d, distance(tr.x, tr.y, x, y))
            if mahalanobis_gate is not None:
                md = tr.mahalanobis(x, y, stamp, r_i)
                if md is not None:
                    if md > mahalanobis_gate:
                        continue
                elif gate_d > gate_radius(stamp - tr.last_stamp, gating_max_speed, min_gate):
                    continue  # 칼만필터가 없는 트랙은 유클리드로 폴백
            elif gate_d > gate_radius(stamp - tr.last_stamp, gating_max_speed, min_gate):
                continue
            sim = cosine_similarity(det_emb, tr.embedding)
            if sim is not None:
                # [외형 거부] 위치 게이트는 통과했어도 외형이 명백히 다르면 아예 후보에서 뺀다.
                # 비용에 더하기만 하면 "후보가 하나뿐일 때"는 아무 견제가 안 돼, 트랙이 다른
                # 사람의 검출로 그대로 끌려간다(실측: my_new_bag4 점프 12건 중 10건이 추종
                # 트랙 id는 그대로인 채 위치만 튀는 경우였다). 임계는 측정된 동일인 5%분위
                # (0.447)보다 낮게 잡아 진짜 같은 사람을 잘라내지 않으면서 명백한 타인만 막는다.
                if appearance_reject is not None and sim < appearance_reject:
                    continue
                if appearance_weight > 0.0:
                    d += appearance_weight * (1.0 - sim)
            cost[i][j] = d

    row_idx, col_idx = linear_sum_assignment(cost)
    for i, j in zip(row_idx, col_idx):
        if cost[i][j] < _UNREACHABLE_COST:
            results[i] = track_ids[j]

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


def extract_person_variance(detection):
    """'person' 가설의 위치 분산(m^2)을 뽑는다. 안 채워져 있으면 None.

    oakd_detector_node는 접지점 산출 등급(GRADE_SIGMA)과 RGB-Depth 동기 시간차로 계산한
    분산을 covariance[0]에 넣어 발행한다 - "검출기가 자기 불확실도를 스스로 신고한다"는
    설계였는데 하류에서 아무도 안 읽고 있었다. 칼만필터의 관측 노이즈 R로 쓰면 프레임별
    신뢰도가 추정과 게이팅에 그대로 반영된다.
    """
    for result in detection.results:
        if result.hypothesis.class_id == 'person':
            var = float(result.pose.covariance[0])
            return var if var > 0.0 else None
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


def sec_to_stamp(sec):
    """초(float) -> builtin_interfaces/Time. stamp_to_sec의 역함수."""
    from builtin_interfaces.msg import Time
    msg = Time()
    msg.sec = int(sec)
    msg.nanosec = int(round((sec - msg.sec) * 1e9))
    if msg.nanosec >= 1000000000:
        msg.sec += 1
        msg.nanosec -= 1000000000
    return msg


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9
