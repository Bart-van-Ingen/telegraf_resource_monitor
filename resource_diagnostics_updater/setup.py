from setuptools import find_packages, setup

package_name = 'resource_diagnostics_updater'

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
    maintainer='dev',
    maintainer_email='bart@adinnovations.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'resource_diagnostics_updater_node = resource_diagnostics_updater.resource_diagnostics_updater_node:main'
        ],
    },
)
