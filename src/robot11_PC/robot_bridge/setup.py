import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    # 동일 구현으로 robot5와 robot11을 연결하는 공통 브릿지 패키지다.
    description='robot5/robot11 중앙 시스템 연동 브릿지 노드',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 기존 robot11 실행 이름은 호환성을 위해 유지하고 robot5 실행 이름을 추가한다.
            'robot5_bridge_node = robot_bridge.robot11_bridge_node:main_robot5',
            'robot11_bridge_node = robot_bridge.robot11_bridge_node:main',
        ],
    },
)
