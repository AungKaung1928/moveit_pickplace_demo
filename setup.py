from setuptools import setup
import os
from glob import glob

package_name = 'simple_moveit_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aung Kaung Myat',
    maintainer_email='aungkaungmyattt1928@gmail.com',
    description='MoveIt2 smart pick-and-place with vision, ArUco, PyTorch, and C++',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Original simple demo (kept)
            'pick_place_demo    = simple_moveit_demo.pick_place_demo:main',
            # Vision pipeline
            'camera_simulator   = simple_moveit_demo.camera_simulator:main',
            'vision_detector    = simple_moveit_demo.vision_detector:main',
            'depth_estimator    = simple_moveit_demo.depth_estimator:main',
            # Smart controller
            'smart_pick_place   = simple_moveit_demo.smart_pick_place:main',
            # Perception tools
            'export_model       = simple_moveit_demo.export_model:main',
            'benchmark          = simple_moveit_demo.benchmark:main',
        ],
    },
)
