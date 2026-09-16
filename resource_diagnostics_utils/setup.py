from pathlib import Path

from setuptools import find_packages, setup


package_name = 'resource_diagnostics_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all config files.
        (
            str(Path('share') / package_name / 'config'),
            [str(config_file_path) for config_file_path in Path('config').glob('*')],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bart',
    maintainer_email='van.ingen.bart@gmail.com',
    description='Python libraries used by other Data-Collection packages',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
