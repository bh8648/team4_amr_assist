import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_manager'

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
    description='Robot DB Manager and Dummy Publisher Node Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'db_manager_node = robot_manager.db_manager_node:main',
            'hmi_backend_node = robot_manager.hmi_backend_node:main',
            'robot_assignment_node = robot_manager.robot_assignment_node:main',
            'task_manager_node = robot_manager.task_manager_node:main',
            'deadlock_prevention_node = robot_manager.deadlock_prevention_node:main',
            # 중앙 DB의 목적지 설정을 로봇별 HMI와 Task Manager에 배포한다.
            'destination_manager_node = robot_manager.destination_manager_node:main',
            'dummy_publisher = robot_manager.dummy_status_publisher:main',
            'webcam_pc_cli = robot_manager.webcam_pc_cli:main',
        ],
    },
)
