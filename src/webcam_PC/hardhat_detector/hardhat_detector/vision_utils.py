# vision_utils.py
#
# person_locator/vision_utils.py의 encode_jpeg/decode_jpeg와 동일한 내용을
# 그대로 복제해둔 것 (hand_gesture_caller/vision_utils.py와 같은 이유 -
# 이 패키지가 person_locator를 의존하지 않고도 "person/face_roi/compressed"
# 토픽의 CompressedImage만으로 완전히 독립적으로 동작하게 하려고).

import cv2  # OpenCV: JPEG 인코딩/디코딩에 사용
import numpy as np  # 바이트 버퍼 <-> ndarray 변환에 사용
from sensor_msgs.msg import CompressedImage  # ROS2 압축 이미지 메시지 타입


def encode_jpeg(frame, quality=80):
    # BGR ndarray 프레임을 지정한 품질로 JPEG 바이트로 인코딩
    ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError('JPEG 인코딩 실패')  # 인코딩 실패 시 바로 예외 발생
    msg = CompressedImage()  # 퍼블리시할 메시지 객체 생성
    msg.format = 'jpeg'  # 포맷 필드에 jpeg 명시
    msg.data = buf.tobytes()  # 인코딩된 바이트를 메시지 payload로 저장
    return msg


def decode_jpeg(msg):
    # 메시지의 바이트 payload를 uint8 버퍼로 재해석
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    # JPEG 버퍼를 BGR 컬러 ndarray 프레임으로 디코딩
    frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    return frame  # 디코딩된 프레임 반환


def classify_hardhat(model, roi_bgr, positive_class_name='helmet', conf_threshold=0.5):
    """얼굴 ROI 하나를 모델에 넣어 "하이바 착용" 여부와 신뢰도를 얻는다.

    ultralytics YOLO 모델을 가정하되, model.task에 따라 두 가지 모델
    형태를 다 지원한다 - pose_locator_node가 이미 같은 라이브러리로
    YOLO-pose를 돌리고 있어서(person_locator/pose_locator_node.py)
    로딩/추론 방식을 통일함:

    - task == 'classify' (예: yolov8n-cls 파인튜닝): ROI 전체를 "하이바"/
      "노하이바" 같은 클래스 하나로 분류. top1 클래스와 confidence를 그대로 씀.
    - task == 'detect' (예: cart/truck/helmet 같은 클래스를 박스로 검출하는
      모델): ROI 안에서 positive_class_name 박스가 검출되면 착용으로 봄.
      박스가 아예 없으면 "미착용"이 이미 확정된 신호이므로 confidence는
      0.0으로 반환함(post-hoc 게이팅 없이 predict()의 conf_threshold로
      이미 걸러진 결과).

    어느 쪽이든 모델 학습 시 클래스 이름을 뭐라고 붙였는지에 맞춰
    positive_class_name만 바꾸면 되고 코드는 건드릴 필요 없음 - 클래스
    순서(index)에 의존하지 않고 모델이 갖고 있는 이름표(model.names)로
    직접 비교함.

    Returns:
        (is_wearing, confidence) 튜플.
    """
    if model.task == 'classify':
        results = model.predict(roi_bgr, verbose=False)  # 분류 모델 추론 실행
        probs = results[0].probs  # 클래스별 확률 객체
        top1_idx = int(probs.top1)  # 가장 확률 높은 클래스 인덱스
        confidence = float(probs.top1conf)  # 그 클래스의 confidence
        class_name = model.names[top1_idx]  # 인덱스 -> 클래스 이름
        is_wearing = (class_name == positive_class_name)  # 이름이 "착용" 클래스와 일치하는지
        return is_wearing, confidence

    if model.task == 'detect':
        results = model.predict(roi_bgr, conf=conf_threshold, verbose=False)  # 검출 모델 추론 실행
        boxes = results[0].boxes  # 검출된 박스들
        best_conf = 0.0  # positive_class_name 박스 중 최고 confidence
        for cls_idx, box_conf in zip(boxes.cls.tolist(), boxes.conf.tolist()):
            if model.names[int(cls_idx)] == positive_class_name:  # 원하는 클래스 박스만 대상
                best_conf = max(best_conf, float(box_conf))
        return best_conf > 0.0, best_conf  # 박스를 하나라도 찾았으면 착용으로 판정

    raise ValueError(f'지원하지 않는 모델 task: {model.task!r} (classify 또는 detect만 지원)')
