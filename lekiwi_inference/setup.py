from setuptools import find_packages, setup
import os
from glob import glob

package_name = "lekiwi_inference"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "msgpack", "pyzmq", "numpy"],
    zip_safe=True,
    maintainer="LeKiwi Robotics Team",
    maintainer_email="dev@lekiwi.labs",
    description="Physical-AI policy inference package for LeKiwi robot (Phase 1: Async ZeroMQ)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "async_ros2_client_node = lekiwi_inference.async_ros2_client_node:main",
        ],
    },
)

