from glob import glob
from setuptools import find_packages, setup

package_name = 'continuum_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wangwenfeng',
    maintainer_email='714394863@qq.com',
    description='Vision perception nodes for continuum robot AprilTag tracking',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_tip_pose_node = continuum_vision.apriltag_tip_pose_node:main',
        ],
    },
)
