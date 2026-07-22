# Copyright 2026 u5-4
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

"""Launch the stage-1 localization adapter in non-publishing shadow mode."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_NAME = 'cuvslam_localization_adapter'


def generate_launch_description() -> LaunchDescription:
    """Create the shadow-only launch description."""
    default_contract = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        'config',
        'contract_blocked.yaml',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'contract_file',
            default_value=default_contract,
            description='Version-controlled localization contract YAML.',
        ),
        Node(
            package=PACKAGE_NAME,
            executable='cuvslam_localization_adapter_node',
            name='cuvslam_localization_adapter',
            output='screen',
            parameters=[{
                'mode': 'shadow',
                'contract_file': LaunchConfiguration('contract_file'),
            }],
        ),
    ])
