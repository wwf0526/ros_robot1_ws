from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

package_name = 'continuum_mujoco_sim'


def only_files(pattern):
    """
    glob(pattern) 会匹配文件和目录。
    setuptools 的 data_files 只能安装普通文件，不能安装目录。
    因此这里过滤掉 __pycache__ 等目录，只保留真实文件。
    """
    return [p for p in glob(pattern) if Path(p).is_file()]


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),

        # 手写 MuJoCo XML
        (
            'share/' + package_name + '/models',
            only_files('models/*.xml'),
        ),

        # MuJoCo 资源文件，例如六边形盘 STL
        # 注意：这里不能使用 glob('models/assets/*') 直接安装所有内容，
        # 否则 __pycache__ 目录也会被当成资源复制，导致 colcon build 失败。
        (
            'share/' + package_name + '/models/assets',
            only_files('models/assets/*'),
        ),

        # 自动生成的 MuJoCo XML
        (
            'share/' + package_name + '/models/generated',
            only_files('models/generated/*.xml'),
        ),
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
            'generate_mujoco_model = continuum_mujoco_sim.generate_mujoco_model:main',
        ],
    },
)
