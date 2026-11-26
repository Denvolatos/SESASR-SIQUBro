"""Setuptools entry point for lab07_pkg."""

from glob import glob

from setuptools import find_packages, setup

package_name = 'lab07_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'scripts']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SIQUBro (SESASR group 203)',
    maintainer_email='siqubro@todo.todo',
    description='DWA obstacle avoidance and dynamic goal following for TurtleBot3',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dwa_node = lab07_pkg.dwa_node:main',
        ],
    },
)
