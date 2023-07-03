from setuptools import setup
from glob import glob
import os

package_name = 'arcros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all XML files from the 'assets' folder
        # Include all launch files.
        (os.path.join('share', package_name, 'assets'), glob(os.path.join('assets', '*.xml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gerald Ebmer',
    maintainer_email='gerald.ebmer@tuwien.ac.at',
    description='ROS2 bridge for arc.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'iiwajoco = arcros2.iiwajoco:main',
            'listener = arcros2.listener:main',
        ],
    },
)
