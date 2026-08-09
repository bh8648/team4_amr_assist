from setuptools import find_packages, setup

package_name = 'hand_gesture_caller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='MediaPipe Hands로 손목 ROI에서 쥐다/펴다 제스처를 인식해 person_locator에 호출 트리거를 보냄',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wrist_gesture_node = hand_gesture_caller.wrist_gesture_node:main',
        ],
    },
)
