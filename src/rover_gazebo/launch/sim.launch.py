#!/usr/bin/env python3

"""
This program is free software: you can redistribute it and/or modify it 
under the terms of the GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. 
If not, see <https://www.gnu.org/licenses/>.

created by Thanacha Choopojcharoen at CoXsys Robotics (2022)
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro    
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource

# for display robot_state_publisher and fix something
    
def generate_launch_description():
    
    pkg = get_package_share_directory('rover_gazebo')
    rviz_path = os.path.join(pkg,'rviz','display.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen')

    # for launch file dummy_robot.launch.py
    dummy_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("rover_description"),
                    "launch",
                    "rover.launch.py"
                )
            ]
        ),
        launch_arguments={"use_sim_time":"true"}.items()
    )

    # launch gazebo app
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    pkg,
                    "launch",
                    "load_world_into_gazebo.launch.py"
                )
            ]
        )
    )

    spawn_entity_default = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "robot",
            '-x', '2.0',
            '-y', '2.07',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '3.14159265359'
        ],
        output="screen"
    )

    spawn_entity_limo_map = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "robot",
            # '-x', '9.073496746393584',
            # '-y', '0.0',
            # '-z', '0.0',
            # '-R', '0.0',
            # '-P', '0.0',
            # '-Y', '1.5700039414375448'
            '-x', '6.683857497627434',
            '-y', '3.509575305683288',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '2.6103058788899838'
        ],
        output="screen"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    velocity_controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controllers", "--controller-manager", "/controller_manager"],
    )

    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
    )

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    # Static Transform Publisher (world -> odom)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "odom"],
        output="screen"
    )

    launch_description = LaunchDescription()

    launch_description.add_action(rviz)
    launch_description.add_action(dummy_robot)
    launch_description.add_action(gazebo)
    # launch_description.add_action(spawn_entity_default)
    launch_description.add_action(spawn_entity_limo_map)
    launch_description.add_action(joint_state_broadcaster_spawner)
    launch_description.add_action(velocity_controllers)
    # launch_description.add_action(joint_trajectory_controller)
    launch_description.add_action(position_controller)
    launch_description.add_action(static_tf)

    
    return launch_description