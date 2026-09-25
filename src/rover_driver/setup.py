from setuptools import find_packages, setup

package_name = 'rover_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arsham Farahani',
    maintainer_email='arshamfn@todo.todo',
    description='ROS2 driver node that bridges /cmd_vel to WaveShare Wave Rover JSON serial protocol',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_driver_node = rover_driver.rover_driver_node:main',
        ],
    },
)
