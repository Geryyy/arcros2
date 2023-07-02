from setuptools import setup

package_name = 'arcros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'testtext = arcros2.hello:main',
            'listener = arcros2.listener:main',
        ],
    },
)
