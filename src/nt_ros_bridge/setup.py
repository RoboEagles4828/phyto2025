from setuptools import find_packages, setup

package_name = 'nt_ros_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pynetworktables'],
    zip_safe=True,
    maintainer='RoboEagles4828',
    maintainer_email='roboeagles4828@gmail.com',
    description='Bridges NetworkTables from RoboRIO to ROS2 Topics',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
