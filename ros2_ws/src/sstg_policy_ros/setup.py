from glob import glob
from setuptools import find_packages, setup


package_name = "sstg_policy_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="SSTG Team",
    maintainer_email="maintainer@example.invalid",
    description="ROS 2 adapter for belief-only SSTG exploration policies.",
    license="LicenseRef-Proprietary-Until-Project-License-Is-Frozen",
    entry_points={
        "console_scripts": [
            "policy_node = sstg_policy_ros.policy_node:main",
        ],
    },
)
