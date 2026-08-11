from setuptools import find_packages, setup  # ament_python 패키지 빌드용 setuptools 함수
import os  # 경로 조합용
from glob import glob  # launch/config/models 디렉터리 파일 목록 수집용

package_name = 'person_locator'  # 패키지 이름 (디렉터리명과 일치해야 함)

setup(
    name=package_name,  # 설치될 패키지 이름
    version='0.0.0',  # 패키지 버전
    packages=find_packages(exclude=['test']),  # test 디렉터리를 제외한 모든 파이썬 패키지 자동 탐색 (calibration 서브패키지 포함)
    data_files=[
        ('share/ament_index/resource_index/packages',  # ROS2가 패키지를 찾을 수 있게 리소스 인덱스 등록
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),  # package.xml을 share 디렉터리로 설치
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # launch 파일들 설치
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.json')),  # 호모그래피 yaml + 파라미터 json 설치
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),  # YOLO-pose 가중치 파일 설치
    ],
    install_requires=['setuptools'],  # 런타임 파이썬 의존성
    zip_safe=True,  # zip 아카이브 형태로 설치해도 안전함을 명시
    maintainer='rokey',  # 패키지 관리자 이름
    maintainer_email='rokey@todo.todo',  # 관리자 이메일
    description='YOLO-pose + homography: locate a standing person in the map frame and publish it for AMR navigation to consume',  # 패키지 설명
    license='Apache-2.0',  # 라이선스
    extras_require={
        'test': [
            'pytest',  # 테스트 실행용 의존성
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_publisher = person_locator.camera_publisher:main',  # 카메라 프레임을 토픽으로 publish
            'calibrate_homography = person_locator.calibration.calibrate_homography:main',  # 호모그래피 캘리브레이션 도구
            'pick_pixel_points = person_locator.calibration.pick_pixel_points:main',  # 픽셀 좌표 클릭 선택 도구
            'log_amr_positions = person_locator.calibration.log_amr_positions:main',  # AMR 실좌표 로깅 도구
            'compute_homography_from_points = person_locator.calibration.compute_homography_from_points:main',  # 대응점으로 호모그래피 계산
            'pose_locator_node = person_locator.pose_locator_node:main',  # 메인 YOLO-pose + homography 노드
            'call_position_nav_test = person_locator.call_position_nav_test:main',  # 호출 위치 Nav2 테스트 도구
        ],
    },
)
