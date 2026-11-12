from glob import glob

from setuptools import find_packages, setup

package_name = 'lab04_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
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
    description='EKF localization of the TurtleBot3 with landmarks, odometry and IMU',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ekf_node = lab04_pkg.ekf_node:main',
            'jin_ekf_node = lab04_pkg.jin_ekf_node:main',
        ],
    },
)
