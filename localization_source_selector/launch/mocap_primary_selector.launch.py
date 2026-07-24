# Copyright (c) 2026 u5-4
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the immutable mocap-primary selector."""
    share = get_package_share_directory("localization_source_selector")
    contract = os.path.join(share, "config", "mocap_primary.contract.yaml")
    return LaunchDescription([
        Node(
            package="localization_source_selector",
            executable="localization_source_selector_node",
            name="localization_source_selector",
            namespace="/",
            output="screen",
            parameters=[{
                "mode": "mocap_primary",
                "contract_file": contract,
            }],
        ),
    ])
