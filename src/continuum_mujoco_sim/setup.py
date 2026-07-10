from glob import glob
from setuptools import find_packages, setup

package_name = 'continuum_mujoco_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', glob('models/*.xml')),
        ('share/' + package_name + '/models/assets', glob('models/assets/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wangwenfeng',
    maintainer_email='714394863@qq.com',
    description='MuJoCo kinematic visualization for continuum robot PCC/MPC validation',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mujoco_pcc_viewer = continuum_mujoco_sim.mujoco_pcc_viewer:main',
            'pcc_section_demo = continuum_mujoco_sim.pcc_section_demo:main',
        ],
    },
)
