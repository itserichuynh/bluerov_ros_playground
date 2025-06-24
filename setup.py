from setuptools import setup
import os
from glob import glob

package_name = 'bluerov_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=['bluerov', 'bridge'],  # both subfolders under src
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # include launch and model files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'model'), glob('model/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eric Huynh',
    maintainer_email='you@example.com',
    description='ROS 2 interface for BlueROV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bluerov_node = bridge.bluerov_node:main',
        ],
    },
)
