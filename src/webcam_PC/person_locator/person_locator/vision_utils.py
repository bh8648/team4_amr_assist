# vision_utils.py
#
# pose_locator_node와 calibrate_homography 툴에서 공통으로 쓰는
# 순수 Python / OpenCV 헬퍼 함수 모음. rclpy 노드를 직접 만들거나 spin하지
# 않으므로 노드를 안 띄우고도 단위 테스트하거나 재사용할 수 있음
# (다만 CompressedImage 메시지 "타입"은 그냥 데이터 클래스라서 import는 함 -
# 이건 실행 중인 노드가 있어야 동작하는 게 아님).
#
# 이 파일 전체에서 쓰는 COCO pose keypoint 순서 (Ultralytics YOLO-pose 출력 순서):
#   0 코, 1 왼쪽눈, 2 오른쪽눈, 3 왼쪽귀, 4 오른쪽귀,
#   5 왼쪽어깨, 6 오른쪽어깨, 7 왼쪽팔꿈치, 8 오른쪽팔꿈치,
#   9 왼쪽손목, 10 오른쪽손목, 11 왼쪽엉덩이, 12 오른쪽엉덩이,
#   13 왼쪽무릎, 14 오른쪽무릎, 15 왼쪽발목, 16 오른쪽발목

import cv2          # OpenCV: homography 계산 + JPEG 인코딩/디코딩
import numpy as np  # 배열 연산
import yaml         # calibrate_homography.py가 저장한 homography YAML 읽기용
from sensor_msgs.msg import CompressedImage  # 카메라 프레임을 실어 나르는 압축 이미지 메시지 타입


def encode_jpeg(frame, quality=80):
    """OpenCV BGR 프레임을 JPEG로 압축해서 CompressedImage 메시지로 만든다.

    raw sensor_msgs/Image(비압축)는 640x480 BGR 기준 한 장에 ~900KB라
    15Hz만 해도 초당 13MB 넘게 나가서, 카메라가 있는 이 PC와 추론이 도는
    다른 PC 사이를 네트워크로 보내기엔 너무 무거움. JPEG로 압축하면
    같은 프레임이 quality=80 기준 보통 수십 KB 수준으로 줄어듦.
    """
    ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError('JPEG 인코딩 실패')
    msg = CompressedImage()
    msg.format = 'jpeg'
    # buf는 (N, 1) 형태의 uint8 numpy 배열 - tobytes()로 메시지의 data
    # 필드(uint8[] / bytes)에 바로 넣을 수 있는 형태로 변환
    msg.data = buf.tobytes()
    return msg


def decode_jpeg(msg):
    """encode_jpeg가 만든 CompressedImage 메시지를 OpenCV BGR 프레임으로 되돌린다."""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    return frame

# 양쪽 발목 keypoint 인덱스. 발목 = "발이 바닥에 닿는 지점"이라서
# 우리가 실제로 map (x, y)로 변환하고 싶은 값은 이거임
# (엉덩이나 코 같은 걸 쓰면 지면 기준 homography를 적용해도 바닥에서 붕 뜬 위치가 나옴)
LEFT_ANKLE_IDX = 15  # COCO keypoint 순서상 왼쪽 발목 인덱스
RIGHT_ANKLE_IDX = 16  # COCO keypoint 순서상 오른쪽 발목 인덱스

# 양쪽 손목 keypoint 인덱스 - wrist_gesture_node에 넘길 손 ROI를 자를 중심점
LEFT_WRIST_IDX = 9  # COCO keypoint 순서상 왼쪽 손목 인덱스
RIGHT_WRIST_IDX = 10  # COCO keypoint 순서상 오른쪽 손목 인덱스



def apply_homography(u, v, H):
    """픽셀 (u, v)를 homography 행렬 H(3x3)로 변환해서 지면 기준 (x, y)를 얻는다.

    rc_car_chase/vision_utils.py 걸 그대로 가져옴 - 이 수식은 범용이라
    (아무 픽셀 -> 지면 좌표) RC카 검출기에서 왔든 사람 발목에서 왔든 상관없음.
    """
    # cv2.perspectiveTransform은 점 하나만 넣어도 (N, 1, 2) 형태의 3차원 배열을 요구함
    # 그래서 아래처럼 대괄호를 두 번 감싸는 것
    pt = np.array([[[float(u), float(v)]]], dtype=np.float32)
    # 3x3 homography 행렬을 적용. 내부적으로 perspective divide(동차좌표 나눗셈)까지
    # 알아서 해주기 때문에 결과는 바로 target 프레임(H를 캘리브레이션할 때 쓴 프레임,
    # 여기서는 "map" 프레임)의 평범한 (x, y)로 나옴
    out = cv2.perspectiveTransform(pt, H)
    # out은 (1, 1, 2) 형태이므로 (x, y) 하나만 꺼내옴
    x, y = out[0, 0]
    return float(x), float(y)


def load_homography_yaml(path):
    """calibrate_homography.py가 저장한 3x3 homography 행렬을 불러온다."""
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    # YAML에는 사람이 읽거나 diff 보기 편하게 9개짜리 리스트(행 우선)로
    # flatten해서 저장해뒀으므로 여기서 다시 3x3으로 reshape
    values = data['homography_matrix']
    H = np.array(values, dtype=np.float64).reshape(3, 3)
    return H


def extract_standing_pixel(keypoints_xy, keypoints_conf, box_xyxy, ankle_conf_threshold=0.5):
    """사람의 발이 바닥에 닿는 지점을 가장 잘 나타내는 픽셀 (u, v)를 고른다.

    Args:
        keypoints_xy: (17, 2) 형태의 array - 이번 검출의 17개 COCO keypoint
            각각의 (x, y) 픽셀 좌표. Ultralytics의 `result.keypoints.xy[i]`.
        keypoints_conf: (17,) 형태의 array - keypoint별 신뢰도.
            `result.keypoints.conf[i]`. 모델이 신뢰도를 안 주면 None일 수도
            있는데, 그 경우엔 바로 bbox fallback으로 감.
        box_xyxy: 같은 검출의 (x1, y1, x2, y2) 픽셀 bounding box.
            `result.boxes.xyxy[i]`.
        ankle_conf_threshold: 발목 keypoint를 믿을 수 있는 최소 신뢰도.
            이보다 낮으면 "가려져서 실제로 안 보이는 발목"으로 간주하고
            (예: 프레임 밖으로 다리가 잘렸거나 가구에 가려짐) 더 안전한
            bbox 기준으로 대체함.

    Returns:
        (u, v) 튜플: 최종적으로 고른 "바닥 접촉" 픽셀.
    """
    # 신뢰도 정보 자체가 없는 경우 방어 - Ultralytics 설정에 따라 keypoint
    # 신뢰도 없이 좌표만 줄 수도 있음. 신뢰도가 없으면 진짜 발목 검출인지
    # 대충 찍은 건지 구분할 수 없으니 바로 더 안전한 bbox 하단 fallback으로 감
    # 왼쪽, 오른쪽발목 인덱스 15, 16을 각각 확인해서 둘 다 신뢰도가 낮으면 fallback으로 감
    if keypoints_conf is not None:
        left_conf = float(keypoints_conf[LEFT_ANKLE_IDX])
        right_conf = float(keypoints_conf[RIGHT_ANKLE_IDX])

        left_ok = left_conf >= ankle_conf_threshold
        right_ok = right_conf >= ankle_conf_threshold

        if left_ok and right_ok:
            # 양쪽 발목 다 확실하게 검출됐으면 두 점의 중점을 "서 있는 위치"로 사용
            # - 발목 하나만 쓰는 것보다 안정적임 (걸음 중간 자세도 평균으로 상쇄됨)
            lu, lv = keypoints_xy[LEFT_ANKLE_IDX]
            ru, rv = keypoints_xy[RIGHT_ANKLE_IDX]
            u = (float(lu) + float(ru)) / 2.0
            v = (float(lv) + float(rv)) / 2.0
            return u, v

        if left_ok:
            # 왼쪽 발목만 믿을 만함 (예: 오른쪽 다리가 가려짐)
            lu, lv = keypoints_xy[LEFT_ANKLE_IDX]
            return float(lu), float(lv)

        if right_ok:
            # 오른쪽 발목만 믿을 만함
            ru, rv = keypoints_xy[RIGHT_ANKLE_IDX]
            return float(ru), float(rv)

    # Fallback: 양쪽 발목 다 신뢰도가 낮음 (또는 신뢰도 정보 자체가 없음).
    # 이 경우 bounding box 하단-중앙을 대신 사용 - rc_car_chase가 RC카에
    # 쓰던 것과 똑같은 "바닥 접촉점" 근사 방식
    # (x1+x2)/2로 가로 중심, y2로 "박스의 맨 아래"를 잡음
    x1, y1, x2, y2 = box_xyxy
    u = (float(x1) + float(x2)) / 2.0
    v = float(y2)
    return u, v


def extract_wrist_pixel(keypoints_xy, keypoints_conf, conf_threshold=0.5, locked_side=None):
    """제스처 인식용으로 손 ROI를 자를 중심 픽셀 (u, v)를 고른다.

    extract_standing_pixel과 달리 bbox fallback이 없다 - 발목은 "바닥에 닿는
    점"을 대충이라도 근사할 수 있지만, 손목은 신뢰도 낮은 keypoint를 그대로
    쓰면 엉뚱한 곳을 크롭해서 손이 아예 안 걸리는 ROI를 만들게 됨. 그래서
    둘 다 신뢰도 미달이면 그냥 None을 반환해서 호출하는 쪽이 이번 프레임은
    건너뛰게 한다.

    locked_side로 select_person(locked_track_id)과 같은 원칙의 락온을 적용함:
    왼쪽/오른쪽 신뢰도가 서로 비슷하면 프레임마다 근소한 차이로 승자가
    바뀌면서 크롭 위치 자체가 왼쪽<->오른쪽으로 계속 튀는 문제가 있었음 -
    이미 락온된 쪽이 있고 그 쪽이 여전히 신뢰도 기준을 넘으면, 다른 쪽이
    이번 프레임에 근소하게 더 높더라도 갈아타지 않고 그대로 유지한다.
    락온된 쪽이 더 이상 안 보일 때만(신뢰도 미달) 그때 다시 둘 중 더 나은
    쪽으로 새로 고른다.

    Args:
        keypoints_xy: (17, 2) 형태의 array - extract_standing_pixel과 동일.
        keypoints_conf: (17,) 형태의 array 또는 None.
        conf_threshold: 손목 keypoint를 믿을 수 있는 최소 신뢰도.
        locked_side: 이전 프레임에 락온했던 쪽('left'/'right'), 아직 락온
            안 했으면 None.

    Returns:
        (u, v, side) 튜플 - side는 이번에 고른 쪽('left'/'right')이라 호출하는
        쪽이 다음 프레임 locked_side로 그대로 넘기면 됨. 신뢰할 만한 손목이
        하나도 없으면 None.
    """
    if keypoints_conf is None:
        return None

    left_conf = float(keypoints_conf[LEFT_WRIST_IDX])
    right_conf = float(keypoints_conf[RIGHT_WRIST_IDX])

    left_ok = left_conf >= conf_threshold
    right_ok = right_conf >= conf_threshold

    # locked_side가 있으면 그 쪽이 이번 프레임에도 신뢰도 기준을 넘는지 확인 -
    # 넘으면 그대로 유지. 안 넘으면 아래에서 새로 고른다
    if locked_side == 'left' and left_ok:
        u, v = keypoints_xy[LEFT_WRIST_IDX]
        return float(u), float(v), 'left'
    if locked_side == 'right' and right_ok:
        u, v = keypoints_xy[RIGHT_WRIST_IDX]
        return float(u), float(v), 'right'

    if not left_ok and not right_ok:
        return None

    # 락온된 쪽이 없거나(첫 검출) 락온된 쪽이 이번 프레임엔 신뢰도 미달 -
    # 둘 중 더 확실하게 보이는 쪽으로 새로 고름. 발목 중점과 달리 두 손목은
    # 서로 멀리 떨어져 있을 수 있어서 평균 내면 손도 아닌 엉뚱한 중간
    # 지점을 크롭하게 되므로 하나만 고른다
    side = 'left' if left_conf >= right_conf else 'right'
    idx = LEFT_WRIST_IDX if side == 'left' else RIGHT_WRIST_IDX
    u, v = keypoints_xy[idx]
    return float(u), float(v), side


def crop_person_bbox(frame, box_xyxy, padding_ratio=0.15, top_ratio=1.0):
    """YOLO-pose가 이미 계산해둔 사람 bounding box를 그대로 크롭한다 (hardhat_detector 입력용).

    처음엔 코/눈/귀 keypoint로 얼굴만 좁게 크롭했었는데, 오버헤드/광각
    카메라처럼 사람이 화면에서 작게 잡히는 환경에서는 얼굴 keypoint
    신뢰도가 너무 낮아서 크롭 자체가 계속 스킵되는 문제가 있었음. 게다가
    hardhat_detector의 학습 데이터(datasets/detect_warn)도 얼굴만 잘라낸
    게 아니라 카메라 전체 장면이었어서, 학습 때 스케일/구도와 비슷하게
    주는 게 정확도에도 더 유리함.

    box_xyxy는 select_person이 이미 골라낸 검출에서 나온 거라(즉 이미
    "사람"으로 확정된 검출) extract_wrist_pixel처럼 신뢰도 체크로 스킵할
    필요가 없음 - 그래서 이 함수는 None을 반환하지 않고, 항상 크롭 결과를
    돌려준다(padding_ratio만큼 위아래좌우 여유를 둬서 하이바가 박스
    상단 경계에 딱 붙어 잘리는 것도 어느 정도 방지).

    top_ratio(<1.0)를 주면 패딩 전에 박스 높이를 위에서부터 그 비율만큼만
    남기고 잘라낸다 - 전신 대신 하이바가 있을 상반신/머리 영역만 남겨서
    다리/배경이 분류기 입력에 섞이는 걸 줄이기 위함(오탐 튜닝용).

    Returns:
        크롭된 프레임(BGR numpy array).
    """
    x1, y1, x2, y2 = box_xyxy  # 원본 bounding box 좌표
    w = x2 - x1  # 박스 너비
    h = y2 - y1  # 박스 높이
    if top_ratio < 1.0:
        y2 = y1 + h * top_ratio  # 위에서부터 top_ratio 비율만큼만 남기고 아래쪽을 잘라냄
        h = y2 - y1  # 잘라낸 만큼 높이 갱신
    pad_x = w * padding_ratio  # 좌우 여유 픽셀 수
    pad_y = h * padding_ratio  # 상하 여유 픽셀 수

    frame_h, frame_w = frame.shape[:2]  # 프레임 전체 크기 (클램프 기준)
    cx1 = max(0, int(x1 - pad_x))   # 상하좌우 경계가 프레임 밖으로 나가면 클램프해서 잘라냄
    cy1 = max(0, int(y1 - pad_y))
    cx2 = min(frame_w, int(x2 + pad_x))
    cy2 = min(frame_h, int(y2 + pad_y))
    return frame[cy1:cy2, cx1:cx2]  # 최종 크롭 결과


def crop_square_roi(frame, center_u, center_v, half_size):
    """frame에서 (center_u, center_v)를 중심으로 한 변 2*half_size인 정사각형을 자른다.

    프레임 경계를 벗어나는 부분은 잘려나가므로(클램프), 손이 프레임 가장자리
    근처에 있으면 결과가 정사각형이 아니라 그보다 작은 직사각형일 수 있다 -
    호출하는 쪽(wrist_gesture_node)은 이미 정사각형을 가정하지 않고 그냥
    있는 그대로 MediaPipe에 넣으므로 문제 없음.

    Returns:
        크롭된 프레임. 중심점이 프레임 밖이면 빈 배열(크기 0)을 반환할 수 있음 -
        호출하는 쪽에서 크기를 확인해야 함.
    """
    h, w = frame.shape[:2]  # 프레임 전체 크기 (클램프 기준)
    u1 = max(0, int(center_u - half_size))  # 왼쪽 경계 (프레임 밖이면 0으로 클램프)
    u2 = min(w, int(center_u + half_size))  # 오른쪽 경계 (프레임 밖이면 w로 클램프)
    v1 = max(0, int(center_v - half_size))  # 위쪽 경계
    v2 = min(h, int(center_v + half_size))  # 아래쪽 경계
    return frame[v1:v2, u1:u2]  # 최종 크롭 결과 (경계 근처면 정사각형이 아닐 수 있음)


def select_person(boxes, locked_track_id):
    """이번 프레임에 어떤 사람을 추적할지 고른다.

    rc_car_chase의 select_target_box와 비슷하지만, pose 모델은 애초에
    "사람" 클래스 하나만 검출하므로 클래스 필터가 필요 없음 - 여러 명 중
    "누구"를 고를지만 결정하면 됨.

    Args:
        boxes: 이번 프레임 검출 결과인 Ultralytics Boxes 객체 (비어있을 수도 있음).
            면적 계산을 위해 각 박스는 .id(트래커 id, None일 수도 있음)와
            .xyxy(픽셀 bbox)가 필요함.
        locked_track_id: 직전 프레임에서 따라가고 있던 트래커 id.
            아직 아무도 락온 안 했으면 None.

    Returns:
        (index, track_id) 튜플. `index`는 `boxes` 안에서 고른 검출의 위치
        (호출하는 쪽에서 같은 index로 keypoints도 찾아 쓸 수 있게),
        `track_id`는 그 검출의 트래커 id (트래커가 id를 못 줬으면 None).
        검출이 아예 없으면 (None, None) 반환.
    """
    if boxes is None or len(boxes) == 0:
        return None, None  # 검출이 아예 없음

    # 이미 락온된 대상이 있으면, 지금 카메라에 더 가까운 다른 사람이 나타났어도
    # 같은 사람(트래커 id 기준)을 계속 따라가는 걸 우선함 - 이렇게 안 하면
    # 두 사람이 서로 스쳐 지나갈 때 추적 TF가 사람 사이를 왔다갔다 튈 수 있음
    if locked_track_id is not None:
        for i in range(len(boxes)):
            box = boxes[i]
            if box.id is not None and int(box.id[0]) == locked_track_id:
                return i, locked_track_id  # 락온했던 id를 이번 프레임에서도 찾음 - 그대로 유지

    # 아직 락온 안 됐거나 (또는 락온했던 id가 이번 프레임에 사라졌으면):
    # bounding box 면적이 가장 큰 검출을 "가장 가까워 보이는 사람"으로 간주해서 선택
    # - 카메라에 가까운 사람일수록 화면에서 차지하는 픽셀 면적이 큼
    best_index = None
    best_area = -1.0
    for i in range(len(boxes)):
        x1, y1, x2, y2 = boxes[i].xyxy[0].tolist()  # i번째 검출의 픽셀 bbox
        area = max(0.0, x2 - x1) * max(0.0, y2 - y1)  # 박스 면적
        if area > best_area:
            best_area = area  # 지금까지 중 가장 큰 면적 갱신
            best_index = i

    if best_index is None:
        return None, None  # (이론상 도달 안 함 - 위에서 이미 빈 boxes는 걸러짐)

    chosen_box = boxes[best_index]
    chosen_id = int(chosen_box.id[0]) if chosen_box.id is not None else None  # 트래커 id (없으면 None)
    return best_index, chosen_id
