import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'imu_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='www',
    maintainer_email='www@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    	'console_scripts': [
        	'dual_imu_serial_node = imu_driver.dual_imu_serial_node:main',
        	'imu1_listener = imu_driver.imu1_listener:main',
        	'imu2_listener = imu_driver.imu2_listener:main',
        	'dual_imu_extract_listener = imu_driver.dual_imu_extract_listener:main',

   	 ],
	},
)
