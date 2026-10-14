from setuptools import find_packages, setup

package_name = 'rover_explore'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/explore.launch.py']),
        ('share/' + package_name + '/config', ['config/explore_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arsham Faghihnasiri',
    maintainer_email='58406708+ArshamFN@users.noreply.github.com',
    description='Rover frontier explorer, exploration launch and parameters, and map autosave.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'explore_autosave = rover_explore.autosave_node:main',
            'rover_explorer = rover_explore.explorer_node:main',
        ],
    },
)
