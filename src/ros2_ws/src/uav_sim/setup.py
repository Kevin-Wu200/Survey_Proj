from setuptools import find_packages, setup

package_name = 'uav_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/uav_sim.launch.py']),
        ('share/' + package_name + '/worlds',
            ['worlds/empty_with_ground.world']),
        ('share/' + package_name + '/models/m300_rtk',
            ['models/m300_rtk/model.sdf',
             'models/m300_rtk/model.config']),
        ('share/' + package_name + '/config',
            ['config/uav_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AirRunway Dev',
    maintainer_email='dev@airunway.local',
    description='UAV 仿真节点 - M300 RTK 四旋翼',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uav_controller = uav_sim.uav_controller:main',
            'uav_camera_sim = uav_sim.uav_camera_sim:main',
            'uav_mission_controller = uav_sim.uav_mission_controller:main',
        ],
    },
)
