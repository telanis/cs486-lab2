from setuptools import setup
import os
from glob import glob

package_name = 'tiana_safety_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
   data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hongrui Zheng, zzangupenn',
    maintainer_email='billyzheng.bz@gmail.com, zzang@seas.upenn.edu',
    description='Skeleton code for Lab 1: Automatic Emergency Braking at University of Pennsylvania',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'tiana_safety_node = tiana_safety_node.tiana_safety_node:main',
    ],
}

)

