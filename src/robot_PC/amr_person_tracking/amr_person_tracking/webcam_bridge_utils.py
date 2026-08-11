#!/usr/bin/env python3
"""webcam_person_bridge_node가 쓰는 순수 함수들.

외부 웹캠 로컬라이저(별도 PC에서 도는 person_locator 패키지)는 자기 PC의 벽시계로
스탬프를 찍는다. 두 머신의 클럭이 어긋나 있으면 이 파이프라인이 **조용히** 망가지는데,
방향에 따라 증상이 정반대라 진단이 어렵다.

  - 웹캠 클럭이 뒤처지면: tracking_utils.Track.update()가 과거 관측을 거부하고
    반환값을 아무도 확인하지 않는다. 토픽은 정상적으로 흐르는데 기여가 0이 된다.
  - 웹캠 클럭이 앞서면: reid_tracking_node.update_tracks()가 마지막에 부르는
    _prune_tracks(msg.header.stamp)가 트랙 테이블을 통째로 지운다. 추종 대상 포함.

그래서 기본 정책은 "원격 스탬프를 믿지 않고 도착 시각으로 다시 찍는다"이다.
노드 클래스와 분리해 둔 이유는 이 판단들을 rclpy 없이 단위테스트하기 위해서다.
"""

import math


class SkewTracker:
    """로컬 시계와 원격 스탬프의 차이를 추적한다.

    편차 추정에 **최솟값**을 쓴다. 편도 지연은 항상 0 이상이므로
    (로컬 - 원격)의 최솟값이 순수 클럭 오프셋에 가장 가깝고, 네트워크 지터는
    전부 그 위에 얹히는 양수 성분이기 때문이다. 평균을 쓰면 지터가 오프셋 추정을
    오염시킨다.
    """

    def __init__(self):
        self.min_offset = None
        self.last_offset = None
        self.samples = 0

    def update(self, remote_sec, local_sec):
        """관측 한 건을 반영하고 현재 오프셋 추정치를 돌려준다."""
        offset = local_sec - remote_sec
        self.last_offset = offset
        if self.min_offset is None or offset < self.min_offset:
            self.min_offset = offset
        self.samples += 1
        return self.min_offset

    @property
    def jitter(self):
        """가장 최근 관측이 오프셋 추정치보다 얼마나 늦었는가(초). 표본이 없으면 None."""
        if self.last_offset is None or self.min_offset is None:
            return None
        return self.last_offset - self.min_offset


def resolve_stamp(policy, remote_sec, local_sec, offset):
    """이 메시지에 실제로 찍을 시각을 정한다.

    policy:
      'receive_time'     - 도착 시각. 원격 클럭을 아예 쓰지 않는다(기본).
      'passthrough'      - 원격 스탬프 그대로. 두 PC가 NTP/chrony로 동기됐을 때만 옳다.
      'offset_corrected' - 원격 스탬프 + 추정 오프셋. 동기가 없지만 지연 분포를
                           보존하고 싶을 때의 절충안.
    """
    if policy == 'passthrough':
        return remote_sec
    if policy == 'offset_corrected':
        return remote_sec + (offset if offset is not None else 0.0)
    return local_sec


def is_message_fresh(remote_sec, local_sec, offset, max_age):
    """백로그로 밀려 도착한 메시지인지 판정한다. max_age <= 0이면 항상 통과.

    receive_time 정책은 도착 시각을 찍으므로, Wi-Fi가 멈췄다 재전송이 몰아치면
    수 초 전 위치가 "지금"으로 둔갑한다. 추정 오프셋을 뺀 실제 지연이 max_age를
    넘으면 버린다.
    """
    if max_age is None or max_age <= 0.0:
        return True
    delay = (local_sec - remote_sec) - (offset if offset is not None else 0.0)
    return delay <= max_age


def is_valid_point(x, y):
    """NaN/inf 좌표를 거른다. 호모그래피가 발산하면 실제로 나온다."""
    return all(math.isfinite(v) for v in (x, y))
