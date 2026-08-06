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
LEFT_ANKLE_IDX = 15
RIGHT_ANKLE_IDX = 16


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
        return None, None

    # 이미 락온된 대상이 있으면, 지금 카메라에 더 가까운 다른 사람이 나타났어도
    # 같은 사람(트래커 id 기준)을 계속 따라가는 걸 우선함 - 이렇게 안 하면
    # 두 사람이 서로 스쳐 지나갈 때 추적 TF가 사람 사이를 왔다갔다 튈 수 있음
    if locked_track_id is not None:
        for i in range(len(boxes)):
            box = boxes[i]
            if box.id is not None and int(box.id[0]) == locked_track_id:
                return i, locked_track_id

    # 아직 락온 안 됐거나 (또는 락온했던 id가 이번 프레임에 사라졌으면):
    # bounding box 면적이 가장 큰 검출을 "가장 가까워 보이는 사람"으로 간주해서 선택
    # - 카메라에 가까운 사람일수록 화면에서 차지하는 픽셀 면적이 큼
    best_index = None
    best_area = -1.0
    for i in range(len(boxes)):
        x1, y1, x2, y2 = boxes[i].xyxy[0].tolist()
        area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        if area > best_area:
            best_area = area
            best_index = i

    if best_index is None:
        return None, None

    chosen_box = boxes[best_index]
    chosen_id = int(chosen_box.id[0]) if chosen_box.id is not None else None
    return best_index, chosen_id
