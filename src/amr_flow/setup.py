from setuptools import setup

package_name = 'amr_flow'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/config', [
            'config/pc1_webcam_worker.yaml',
            'config/pc2_goal_coordinator.yaml',
            'config/pc3_amr_executor.yaml',
            'config/pc3_robot_status.yaml',
        ]),
        (f'share/{package_name}/launch', [
            'launch/simple_scenario.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='codex',
    maintainer_email='codex@example.com',
    description='Simple PC1-PC2-PC3 AMR flow nodes.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pc1_webcam_worker_node = amr_flow.pc1_webcam_worker_node:main',
            'pc2_goal_coordinator_node = amr_flow.pc2_goal_coordinator_node:main',
            'pc3_amr_executor_node = amr_flow.pc3_amr_executor_node:main',
            'pc3_robot_status_node = amr_flow.pc3_robot_status_node:main',
        ],
    },
)
