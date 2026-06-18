from setuptools import find_packages, setup

package_name = 'ugv_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/ugv_sim.launch.py']),
        ('share/' + package_name + '/worlds',
            ['worlds/empty_with_ground.world']),
        ('share/' + package_name + '/models/ugv_chassis',
            ['models/ugv_chassis/model.sdf',
             'models/ugv_chassis/model.config']),
        ('share/' + package_name + '/config',
            ['config/ugv_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AirRunway Dev',
    maintainer_email='dev@airunway.local',
    description='UGV 仿真节点 - 四轮差速底盘',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ugv_controller = ugv_sim.ugv_controller:main',
            'ugv_sensor_pub = ugv_sim.ugv_sensor_pub:main',
        ],
    },
)
