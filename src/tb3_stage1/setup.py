import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'tb3_stage1'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ↓↓↓ 只有这五行是你要加的 ↓↓↓
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*.pgm') + glob('maps/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),   glob('rviz/*.rviz')),
        # 下面是我自己的TB3模型的增加的
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'models', 'turtlebot3_burger_rgbd'),
            glob('models/turtlebot3_burger_rgbd/*')),
            
            
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Peiyan Xie',
    maintainer_email='StrongerOK@outlook.com',
    description='第一阶段任务：TurtleBot3 自建世界 SLAM 建图与自主导航',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoint_navigator = tb3_stage1.waypoint_navigator:main'
        ],
    },
)