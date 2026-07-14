import sys
from setuptools import find_packages, setup

# English comments in code as requested by the user:
# Workaround to fix incompatibilities between colcon and certain setuptools versions
# by stripping out unrecognized option flags from sys.argv before calling setup()
def remove_arg_with_value(arg_name):
    while arg_name in sys.argv:
        try:
            idx = sys.argv.index(arg_name)
            sys.argv.pop(idx)  # Remove the option flag itself
            if idx < len(sys.argv):
                sys.argv.pop(idx)  # Remove its associated value
        except ValueError:
            break

remove_arg_with_value('--build-directory')
remove_arg_with_value('--install-layout')

if '--editable' in sys.argv:
    sys.argv.remove('--editable')


package_name = 'lerobot_camera_nodes'

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
    maintainer='duykhongcay',
    maintainer_email='nguyendangduy7112003@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lerobot_steoreo_cam = lerobot_camera_nodes.lerobot_steoreo_cam:main'
        ],
    },
)

# English comments in code as requested by the user:
# Post-setup hook to create a compatibility symlink in the 'lib' folder.
# This ensures "ros2 run" can find the executable since setuptools installs it into 'bin'
# due to the lack of 'script_dir' support in modern develop commands.
import os

setup_file_path = os.path.abspath(__file__)
package_dir = os.path.dirname(setup_file_path)
lekiwi_ros2_dir = os.path.dirname(package_dir)
ros2_ws_dir = os.path.dirname(lekiwi_ros2_dir)
install_dir = os.path.join(ros2_ws_dir, 'install', package_name)

try:
    bin_path = os.path.join(install_dir, 'bin', 'lerobot_steoreo_cam')
    lib_dir = os.path.join(install_dir, 'lib', package_name)
    lib_path = os.path.join(lib_dir, 'lerobot_steoreo_cam')
    
    if os.path.exists(bin_path):
        os.makedirs(lib_dir, exist_ok=True)
        if os.path.exists(lib_path) or os.path.islink(lib_path):
            os.remove(lib_path)
        os.symlink('../../bin/lerobot_steoreo_cam', lib_path)
        print(f"Successfully created ROS 2 runner symlink: {lib_path} -> {bin_path}")
except Exception as e:
    print(f"Warning: Failed to create compatibility symlink: {e}")

