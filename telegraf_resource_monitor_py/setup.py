from pathlib import Path

from setuptools import find_packages, setup

package_name = 'telegraf_resource_monitor_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (
            str(Path('share') / package_name / 'launch'),
            [str(launch_file_path) for launch_file_path in Path('launch').glob('*')],
        ),
        # Include the telegraf config so the launch files can find it in the install space
        # rather than through a hardcoded source-tree path.
        (
            str(Path('share') / package_name / 'config'),
            [str(config_file_path) for config_file_path in Path('config').glob('*')],
        ),
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
            ('telegraf_resource_monitor_node ='
             'telegraf_resource_monitor_py.telegraf_resource_monitor_node:main')
        ],
    },
)
