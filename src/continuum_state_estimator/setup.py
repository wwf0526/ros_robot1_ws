from setuptools import find_packages, setup

package_name = 'continuum_state_estimator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wangwenfeng',
    maintainer_email='714394863@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'state_estimator_node = continuum_state_estimator.state_estimator_node:main',
            'imu_zero_check_node = continuum_state_estimator.imu_zero_check_node:main',
        ],
    },
)
