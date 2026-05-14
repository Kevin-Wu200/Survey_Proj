from setuptools import find_packages, setup

package_name = 'fusion_pipeline'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/fusion_pipeline.launch.py']),
        ('share/' + package_name + '/config', ['config/fusion_pipeline_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Survey Team',
    maintainer_email='dev@air-ground-survey.com',
    description='空地协同数据融合管道',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_rectify_node = fusion_pipeline.image_rectify_node:main',
            'pointcloud_filter_node = fusion_pipeline.pointcloud_filter_node:main',
            'sfm_node = fusion_pipeline.sfm_node:main',
            'icp_registration_node = fusion_pipeline.icp_registration_node:main',
            'meshing_node = fusion_pipeline.meshing_node:main',
        ],
    },
)
