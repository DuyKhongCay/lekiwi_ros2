# Copyright 2026 LeKiwi Robotics Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Kinematic and image conversion utilities supporting inference data ingestion."""

from typing import Optional

import numpy as np
from sensor_msgs.msg import CompressedImage, Image, JointState


def extract_joint_positions(
    msg: JointState,
    expected_joints: list[str],
) -> Optional[np.ndarray]:
    """Extracts and orders target joint angles according to policy motor indexing."""
    if not msg.name or not msg.position:
        return None

    name_to_pos = dict(zip(msg.name, msg.position))
    positions = []
    for joint in expected_joints:
        if joint not in name_to_pos:
            return None
        positions.append(float(name_to_pos[joint]))

    return np.array(positions, dtype=np.float32)


def image_msg_to_bytes(msg: Image | CompressedImage) -> bytes:
    """Extracts raw or compressed image bytes directly without redundant decoding."""
    if isinstance(msg, CompressedImage):
        return bytes(msg.data)
    elif isinstance(msg, Image):
        return bytes(msg.data)
    else:
        raise TypeError(f"Unsupported message type: {type(msg)}")

