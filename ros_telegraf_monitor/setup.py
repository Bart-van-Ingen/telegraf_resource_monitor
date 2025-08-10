from setuptools import setup
import os
from glob import glob

package_name = "ros_telegraf_monitor" 

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include all launch files.
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="bart",
    maintainer_email="bart@adinnovations.nl",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "telgraf_monitor_node = ros_telegraf_monitor.telgraf_monitor_node:main"
        ],
    },
)
