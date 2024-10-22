#
# Copyright 2023 BobRos
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
#
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler
from launch.actions import EmitEvent
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.substitutions import FindExecutable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # use gpt config file if provided
    launch_config_yaml = DeclareLaunchArgument('config_yaml', 
        default_value=TextSubstitution(
            text=os.path.join(
                get_package_share_directory("rosgpt4all"),
                "config", "gpt.yaml")))

    # used namespace for the nodes
    launch_ns = DeclareLaunchArgument('ns', 
        default_value=TextSubstitution(text='/gpt'))

    # respawn nodes if exiting abnormal
    launch_respawn = DeclareLaunchArgument('respawn', 
        default_value="false")

    # launch with terminal
    launch_terminal = DeclareLaunchArgument('terminal', 
        default_value="false")

    # nodes

    gpt = Node(
        package='rosgpt4all',
        executable='gpt',
        name='gpt',
        respawn=LaunchConfiguration('respawn'),
        namespace=LaunchConfiguration('ns'),
        output='screen',
        parameters=[LaunchConfiguration('config_yaml')]
    )

    terminal = Node(
        condition=IfCondition(LaunchConfiguration("terminal")),
        package='rosgpt4all',
        executable='terminal',
        name='terminal',
        respawn=LaunchConfiguration('respawn'),
        namespace=LaunchConfiguration('ns'),
        output='screen',
        parameters=[LaunchConfiguration('config_yaml')],
        remappings=[('input', 'gpt_generator')]
    )

    rqt_gui = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("terminal")),
        cmd=[[
            FindExecutable(name='rqt')
        ]],
        shell=True
    )

    return LaunchDescription([
        launch_config_yaml,
        launch_ns,
        launch_respawn,
        launch_terminal,
        gpt,
        terminal,
        rqt_gui,
        RegisterEventHandler( # Shutdown if gpt ends
            OnProcessExit(
                target_action=gpt,
                on_exit=[
                    EmitEvent(event=Shutdown(reason='gpt ended'))
                ]
            )
        )
    ])
