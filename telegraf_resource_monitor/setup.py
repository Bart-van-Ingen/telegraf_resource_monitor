import os
from glob import glob

from setuptools import setup

package_name = 'telegraf_resource_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bart',
    maintainer_email='van.ingen.bart@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telegraf_resource_monitor ='
            ' telegraf_resource_monitor.telegraf_resource_monitor_node:main'
        ],
    },
)
