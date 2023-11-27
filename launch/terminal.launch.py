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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution

def generate_launch_description():

    # use gpt config file if provided
    launch_config_yaml = DeclareLaunchArgument('config_yaml', 
        default_value=TextSubstitution(text=''))
    
    # used namespace for the node
    launch_ns = DeclareLaunchArgument('ns', 
        default_value=TextSubstitution(text='/'))
    
    # used name for the node
    launch_name = DeclareLaunchArgument('name', 
        default_value=TextSubstitution(text='terminal'))

    # input topic for the node
    launch_input = DeclareLaunchArgument('input', 
        default_value=TextSubstitution(text='gpt_generator'))

    # output topic for the node
    launch_output = DeclareLaunchArgument('output', 
        default_value=TextSubstitution(text='gpt_in'))

    # respawn node if exiting abnormal
    launch_respawn = DeclareLaunchArgument('respawn', 
        default_value="false")

    # nodes

    terminal = Node(
        package='rosgpt4all',
        executable='terminal',
        name=LaunchConfiguration('name'),
        namespace=LaunchConfiguration('ns'),
        output='screen',
        parameters=[LaunchConfiguration('config_yaml')],
        remappings=[
            ('input', 'gpt_generator'),
            ('output', 'gpt_in')
        ]
    )

    return LaunchDescription([
        launch_config_yaml,
        launch_ns,
        launch_name,
        launch_input,
        launch_output,
        launch_respawn,
        terminal
    ])
