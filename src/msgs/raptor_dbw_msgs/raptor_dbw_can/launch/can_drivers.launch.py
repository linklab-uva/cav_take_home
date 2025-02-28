# Copyright 2020-2021, The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch file for IAC vehicle."""

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler

from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.conditions import IfCondition


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():
    """Launch all packages for the vehicle in IAC."""

    launch_arguments = [
        DeclareLaunchArgument("interval_sec", default_value="0.01"),
        DeclareLaunchArgument("auto_configure", default_value="true"),
        DeclareLaunchArgument("auto_activate", default_value="true"),
        DeclareLaunchArgument("timeout_sec", default_value="0.01"),
    ]

    node_arguments = []

    # SOCKET CAN RECEVIERS/SENDER
    rx_interfaces = ["can0", "can1"]
    tx_interfaces = ["can0"]
    dbc_files = ["CAN1-INDY-V17", "CAN2-INDY-V0"]

    assert len(rx_interfaces) == len(dbc_files)

    # RAPTOR RECEIVE
    for can in rx_interfaces:
        raptor_rx = LifecycleNode(
            package="ros2_socketcan",
            executable="socket_can_receiver_node_exe",
            name=f"socket_can_receiver_{can}",
            namespace=TextSubstitution(text=""),
            parameters=[
                {"interface": can, "interval_sec": LaunchConfiguration("interval_sec")}
            ],
            remappings=[
                ("/from_can_bus", f"/raptor_dbw_interface/{can}_rx"),
            ],
            output="screen",
        )

        raptor_rx_configure_event_handler = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=raptor_rx,
                on_start=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(raptor_rx),
                            transition_id=Transition.TRANSITION_CONFIGURE,
                        ),
                    ),
                ],
            ),
            condition=IfCondition(LaunchConfiguration("auto_configure")),
        )

        raptor_rx_activate_event_handler = RegisterEventHandler(
            event_handler=OnStateTransition(
                target_lifecycle_node=raptor_rx,
                start_state="configuring",
                goal_state="inactive",
                entities=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(raptor_rx),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        ),
                    ),
                ],
            ),
            condition=IfCondition(LaunchConfiguration("auto_activate")),
        )

        node_arguments.append(raptor_rx)
        node_arguments.append(raptor_rx_configure_event_handler)
        node_arguments.append(raptor_rx_activate_event_handler)

    # RAPTOR TRANSMIT
    for can in tx_interfaces:
        raptor_tx = LifecycleNode(
            package="ros2_socketcan",
            executable="socket_can_sender_node_exe",
            name=f"socket_can_sender_{can}",
            namespace=TextSubstitution(text=""),
            parameters=[
                {
                    "interface": can,
                    "timeout_sec": LaunchConfiguration("timeout_sec"),
                }
            ],
            remappings=[
                ("/to_can_bus", f"/raptor_dbw_interface/{can}_tx"),
            ],
            output="screen",
        )

        raptor_tx_configure_event_handler = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=raptor_tx,
                on_start=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(raptor_tx),
                            transition_id=Transition.TRANSITION_CONFIGURE,
                        ),
                    ),
                ],
            ),
            condition=IfCondition(LaunchConfiguration("auto_configure")),
        )

        raptor_tx_activate_event_handler = RegisterEventHandler(
            event_handler=OnStateTransition(
                target_lifecycle_node=raptor_tx,
                start_state="configuring",
                goal_state="inactive",
                entities=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(raptor_tx),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        ),
                    ),
                ],
            ),
            condition=IfCondition(LaunchConfiguration("auto_activate")),
        )

        node_arguments.append(raptor_tx)
        node_arguments.append(raptor_tx_configure_event_handler)
        node_arguments.append(raptor_tx_activate_event_handler)

    # RAPTOR DRIVER
    parameter_dict = {}
    for idx, can in enumerate(rx_interfaces):
        share_file = get_share_file(
            package_name="raptor_dbw_can", file_name=f"launch/{dbc_files[idx]}.dbc"
        )
        parameter_dict[f"dbw_dbc_file_{can}"] = share_file

    can_frame_to_msg = Node(
        package="raptor_dbw_can",
        executable="can_frame_to_msg_node",
        output="screen",
        namespace="raptor_dbw_interface",
        parameters=[parameter_dict],
    )

    node_arguments.append(can_frame_to_msg)

    return LaunchDescription(launch_arguments + node_arguments)
