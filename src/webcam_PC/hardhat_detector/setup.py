import os  # 경로 조합용
from glob import glob  # launch/models 디렉터리 파일 목록 수집용

from setuptools import find_packages, setup  # ament_python 패키지 빌드용 setuptools 함수

package_name = 'hardhat_detector'  # 패키지 이름 (디렉터리명과 일치해야 함)

setup(
    name=package_name,  # 설치될 패키지 이름
    version='0.0.0',  # 패키지 버전
    packages=find_packages(exclude=['test']),  # test 디렉터리를 제외한 모든 파이썬 패키지 자동 탐색
    data_files=[
        ('share/ament_index/resource_index/packages',  # ROS2가 패키지를 찾을 수 있게 리소스 인덱스 등록
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),  # package.xml을 share 디렉터리로 설치
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),  # launch 파일들 설치
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),  # 모델 가중치 파일들 설치
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),  # 파라미터 기본값 json 설치
    ],
    install_requires=['setuptools'],  # 런타임 파이썬 의존성
    zip_safe=True,  # zip 아카이브 형태로 설치해도 안전함을 명시
    maintainer='rokey',  # 패키지 관리자 이름
    maintainer_email='rokey@todo.todo',  # 관리자 이메일
    description='얼굴 ROI를 분류 모델로 판별해 하이바 착용 여부(Bool)를 AMR에 넘겨줌',  # 패키지 설명
    license='Apache-2.0',  # 라이선스
    extras_require={
        'test': [
            'pytest',  # 테스트 실행용 의존성
        ],
    },
    entry_points={
        'console_scripts': [
            # `ros2 run hardhat_detector hardhat_detector_node`로 실행 가능하게 등록
            'hardhat_detector_node = hardhat_detector.hardhat_detector_node:main',
        ],
    },
)
