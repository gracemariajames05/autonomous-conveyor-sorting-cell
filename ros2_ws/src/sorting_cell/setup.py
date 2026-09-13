from setuptools import find_packages, setup

package_name = 'sorting_cell'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Grace Maria James',
    maintainer_email='gracemariajames05@gmail.com',
    description='ROS 2 robotic arm subsystem for Autonomous Conveyor Sorting Cell',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_controller = sorting_cell.arm_controller:main',
        ],
    },
)
