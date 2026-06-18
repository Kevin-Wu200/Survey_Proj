from setuptools import find_packages, setup

package_name = 'ground_station'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/ground_station.launch.py']),
        ('share/' + package_name + '/config',
            ['config/ground_station_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AirRunway Dev',
    maintainer_email='dev@airunway.local',
    description='地面站数据处理管道',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_receiver = ground_station.data_receiver:main',
            'bag_recorder = ground_station.bag_recorder:main',
            'cache_manager = ground_station.cache_manager:main',
            'ros2_websocket_bridge = ground_station.ros2_websocket_bridge:main',
        ],
    },
)
