import os
from glob import glob
from setuptools import find_packages, setup

package_name = "lekiwi_calibration"

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
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="duykhongcay",
    maintainer_email="duykhongcay@todo.todo",
    description="Unified calibration suite for LeKiwi robot",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "calibrate_chessboard = lekiwi_calibration.chessboard.calibrator_node:main",
            "calibrate_omni_base = lekiwi_calibration.omni.omni_base_calibrator_node:main",
            "calibrate_handeye = lekiwi_calibration.handeye.handeye_calibration_node:main",
            "generate_charuco = lekiwi_calibration.handeye.generate_charuco:main",
        ],
    },
)
