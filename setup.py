from setuptools import find_packages, setup

package_name = 'morphobot_autonomy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zbook',
    maintainer_email='zbook@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_processor = morphobot_autonomy.lidar_processor:main',
            'directional_profile = morphobot_autonomy.directional_profile:main',
            'direction_scorer = morphobot_autonomy.direction_scorer:main',
            'decision_stabilizer = morphobot_autonomy.decision_stabilizer:main',
            'ugv_fsm = morphobot_autonomy.ugv_fsm:main',
            'adaptive_controller = morphobot_autonomy.adaptive_controller:main',
            'twist_to_wheels = morphobot_autonomy.twist_to_wheels:main',
        ],
    },
)
