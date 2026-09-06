import os
from glob import glob
from setuptools import find_packages, setup

package_name = "handeye_calibration"

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
    description="Lightweight Hand-Eye calibration for LeKiwi robot with native GUI.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "handeye_calibration_node = handeye_calibration.handeye_calibration_node:main",
            "generate_charuco = handeye_calibration.generate_charuco:main",
        ],
    },
)
