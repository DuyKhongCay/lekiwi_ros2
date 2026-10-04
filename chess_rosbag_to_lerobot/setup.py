from setuptools import find_packages, setup

package_name = "chess_rosbag_to_lerobot"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [
            "config/lekiwi_chess.yaml",
            "config/lekiwi_chess_dev.yaml",
        ]),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "pyyaml",
        "imageio",
    ],
    zip_safe=True,
    maintainer="DuyKhongCay",
    maintainer_email="duykhongcay@lekiwi.labs",
    description="Convert LeKiwi chess episodes (MCAP rosbag) into LeRobot v3.0 datasets",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "chess_convert = chess_rosbag_to_lerobot.cli:main",
        ],
    },
)
