from setuptools import setup
import os
from glob import glob

package_name = 'data_collection_utils'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Annika Frye',
    maintainer_email='annika@frenchfryes.com',
    description='Tools for collecting and visualizing camera and lidar data through ROS 2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_record_live = scripts.camera_record_live:main'
        ],
    },
)
