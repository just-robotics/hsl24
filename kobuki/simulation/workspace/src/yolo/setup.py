import os

from setuptools import find_packages, setup
from glob import glob

package_name = 'yolo'
submodules = 'yolo/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='artem-kondratew',
    maintainer_email='artemkondratev5@gmail.com',
    description='Semantic segmentation for dynamic SLAM',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_detector = yolo.motion_detector:main',
            'keypoints_visualizer = yolo.keypoints_visualizer:main',
            'fake_fast_detector = yolo.fake_fast_detector:main',
            'aruco_detector = yolo.aruco_detector:main'
        ],
    },
)
