from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'person_locator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='YOLO-pose + homography: locate a standing person in the map frame and broadcast a TF for AMR navigation to consume',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_publisher = person_locator.camera_publisher:main',
            'calibrate_homography = person_locator.calibrate_homography:main',
            'pick_pixel_points = person_locator.pick_pixel_points:main',
            'log_amr_positions = person_locator.log_amr_positions:main',
            'compute_homography_from_points = person_locator.compute_homography_from_points:main',
            'pose_locator_node = person_locator.pose_locator_node:main',
            'person_tf_broadcaster_node = person_locator.person_tf_broadcaster_node:main',
            'call_position_nav_test = person_locator.call_position_nav_test:main',
        ],
    },
)
