# vision_utils.py
#
# person_locator/vision_utils.py의 encode_jpeg/decode_jpeg와 동일한 내용을
# 그대로 복제해둔 것. 이 패키지가 person_locator를 굳이 의존하지 않고도
# (ament_python 패키지 간 의존은 설치 순서/버전 문제를 야기하기 쉬움)
# person/wrist_roi/compressed 토픽의 CompressedImage만으로 완전히 독립적으로
# 동작하게 하려고 일부러 작은 유틸 두 개만 복사해옴.

import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage


def encode_jpeg(frame, quality=80):
    ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError('JPEG 인코딩 실패')
    msg = CompressedImage()
    msg.format = 'jpeg'
    msg.data = buf.tobytes()
    return msg


def decode_jpeg(msg):
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    return frame
