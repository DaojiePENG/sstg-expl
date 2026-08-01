from glob import glob
from setuptools import find_packages, setup


package_name = "sstg_baseline_adapter"

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
    description="Experiment-contract adapters for pinned public ROS 2 baselines.",
    license="LicenseRef-Proprietary-Until-Project-License-Is-Frozen",
    entry_points={
        "console_scripts": [
            "frontier_action_adapter = "
            "sstg_baseline_adapter.frontier_action_adapter:main",
        ],
    },
)
