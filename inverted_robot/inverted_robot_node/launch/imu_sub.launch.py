# Copyright 2023 Ar-Ray-code.
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

from launch import LaunchDescription
from launch.actions import (
    EmitEvent, LogInfo, RegisterEventHandler,
    Shutdown
)
from launch.events import matches_action

from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    inverted_robot_dir = get_package_share_directory('inverted_robot_description')

    inverted_robot_ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            inverted_robot_dir, 'launch', 'inverted_robot.launch.py')),
    )

    imu_node = LifecycleNode(package='rt_usb_9axisimu_driver',
                             executable='rt_usb_9axisimu_driver', namespace='', name='imu_node')

    imu_activate_init = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(imu_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    on_inactivate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=imu_node, goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(imu_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    imu_sub_node = Node(
        package='inverted_robot_node',
        executable='imu_sub_exec',
        output='screen',
        name='imu_sub_node',
        namespace='',
    )

    control_node = Node(
        package='inverted_robot_node',
        executable='control_exec',
        output='screen',
        name='control_node',
        namespace='',
    )


    ld.add_action(inverted_robot_ros2_control)

    ld.add_action(imu_node)
    ld.add_action(imu_activate_init)
    ld.add_action(on_inactivate)
    # ld.add_action(imu_sub_node)
    ld.add_action(control_node)

    return ld
