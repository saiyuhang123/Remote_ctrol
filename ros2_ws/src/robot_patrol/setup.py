from glob import glob
from setuptools import setup

package_name = "robot_patrol"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="dev@example.com",
    description="Local route and multi-point patrol executor",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "patrol_node = robot_patrol.node:main",
            "patrol_guard = robot_patrol.guard:main",
            "inspection_recorder = robot_patrol.inspection_recorder:main",
            "patrol_cli = robot_patrol.cli:main",
        ],
    },
)
