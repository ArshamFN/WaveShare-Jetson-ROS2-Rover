from setuptools import find_packages, setup

package_name = 'pendant_bridge'

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
    maintainer='Arsham Faghihnasiri',
    maintainer_email='58406708+ArshamFN@users.noreply.github.com',
    description='Pendant control node that starts and stops the rover bringup and Nav2 systemd user services, reports their state, and saves maps.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'control_node = pendant_bridge.control_node:main',
        ],
    },
)
