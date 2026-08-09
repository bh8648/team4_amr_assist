import os
from glob import glob
from setuptools import find_packages, setup

# robot11 패키지와 별도로 빌드·배포할 robot5 전용 패키지 이름이다.
package_name = 'robot5_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='robot5 PC 전용 중앙 시스템 연동 브릿지 노드',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        # robot5 전용 파일의 main만 설치한다.
        'robot5_bridge_node = robot5_bridge.robot5_bridge_node:main',
    ]},
)
