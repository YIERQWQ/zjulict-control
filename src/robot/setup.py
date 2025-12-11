from setuptools import setup
import os
from glob import glob

package_name = 'robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # 1. 必要的ROS2元数据
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 2. Launch文件 (必须添加！)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        # 3. URDF文件 (必须添加！)
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        
        # 4. Meshes 模型文件
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        
        # 5. Config 配置文件 (改为包含所有文件，以防漏掉 .world 文件)
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yixi',
    maintainer_email='your_email@example.com',
    description='Robot package for zjunlict',
    license='Apache-2.0',
    tests_require=['pytest'],
    # 关键：声明可执行节点
    entry_points={
        'console_scripts': [
            'test_motion = robot.motion:main',
            'monitor = robot.monitor:main',
        ],
    },
)