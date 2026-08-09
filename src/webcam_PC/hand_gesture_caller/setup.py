from setuptools import find_packages, setup  # ament_python 패키지 빌드용 setuptools 함수

package_name = 'hand_gesture_caller'  # 패키지 이름 (디렉터리명과 일치해야 함)

setup(
    name=package_name,  # 설치될 패키지 이름
    version='0.0.0',  # 패키지 버전
    packages=find_packages(exclude=['test']),  # test 디렉터리를 제외한 모든 파이썬 패키지 자동 탐색
    data_files=[
        ('share/ament_index/resource_index/packages',  # ROS2가 패키지를 찾을 수 있게 리소스 인덱스 등록
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),  # package.xml을 share 디렉터리로 설치
    ],
    install_requires=['setuptools'],  # 런타임 파이썬 의존성
    zip_safe=True,  # zip 아카이브 형태로 설치해도 안전함을 명시
    maintainer='rokey',  # 패키지 관리자 이름
    maintainer_email='rokey@todo.todo',  # 관리자 이메일
    description='MediaPipe Hands로 손목 ROI에서 쥐다/펴다 제스처를 인식해 person_locator에 호출 트리거를 보냄',  # 패키지 설명
    license='Apache-2.0',  # 라이선스
    extras_require={
        'test': [
            'pytest',  # 테스트 실행용 의존성
        ],
    },
    entry_points={
        'console_scripts': [
            # `ros2 run hand_gesture_caller wrist_gesture_node`로 실행 가능하게 등록
            'wrist_gesture_node = hand_gesture_caller.wrist_gesture_node:main',
        ],
    },
)
