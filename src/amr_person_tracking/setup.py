import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'amr_person_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='AMR(TurtleBot4 + OAK-D PRO) 기반 사람 추종 - 근접 3D 위치추정(YOLO-pose+depth), 라이다 다리검출 편입, 재식별/트래킹, 예측적 회피',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'oakd_detector_node = amr_person_tracking.oakd_detector_node:main',
            'debug_viewer_node = amr_person_tracking.debug_viewer_node:main',
            'leg_detector_bridge_node = amr_person_tracking.leg_detector_bridge_node:main',
            'reid_tracking_node = amr_person_tracking.reid_tracking_node:main',
            'predictive_avoidance_node = amr_person_tracking.predictive_avoidance_node:main',
            'depth_view_republisher_node = amr_person_tracking.depth_view_republisher_node:main',
        ],
    },
)
