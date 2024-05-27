import os
from glob import glob
from setuptools import setup

package_name = 'px4_offboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'visualize.rviz']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kartik',
    maintainer_email='kpant14@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'offboard_control_sitl = px4_offboard.offboard_control_sitl:main',
                'offboard_control_swarm = px4_offboard.offboard_control_swarm:main',
                'offboard_control_real = px4_offboard.offboard_control_real:main',
                'visualizer = px4_offboard.visualizer:main',
                'visualizer_swarm = px4_offboard.visualizer_swarm:main',
                'visualizer_camera_network = px4_offboard.visualizer_camera_network:main',
        ],
    },
)
