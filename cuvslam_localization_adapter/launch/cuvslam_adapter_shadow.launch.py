# Copyright (c) 2026 u5-4
# SPDX-License-Identifier: Apache-2.0
"""Launch the stage-1 localization adapter in non-publishing shadow mode."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_NAME = "cuvslam_localization_adapter"


def generate_launch_description() -> LaunchDescription:
    """Create the shadow-only launch description."""
    default_contract = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        "config",
        "contract_blocked.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "contract_file",
            default_value=default_contract,
            description="Version-controlled localization contract YAML.",
        ),
        Node(
            package=PACKAGE_NAME,
            executable="cuvslam_localization_adapter_node",
            name="cuvslam_localization_adapter",
            output="screen",
            parameters=[{
                "mode": "shadow",
                "contract_file": LaunchConfiguration("contract_file"),
            }],
        ),
    ])
