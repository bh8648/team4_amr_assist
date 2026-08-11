# vision_utils.py
#
# person_locator/vision_utils.py의 encode_jpeg/decode_jpeg와 동일한 내용을
# 그대로 복제해둔 것. 이 패키지가 person_locator를 굳이 의존하지 않고도
# (ament_python 패키지 간 의존은 설치 순서/버전 문제를 야기하기 쉬움)
# person/wrist_roi/compressed 토픽의 CompressedImage만으로 완전히 독립적으로
# 동작하게 하려고 일부러 작은 유틸 두 개만 복사해옴.

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
