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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# for display robot_state_publisher and fix something
    
def generate_launch_description():
    
    pkg_rover_gazebo = get_package_share_directory('rover_gazebo')

    # Declare launch argument for steering mode
    steering_mode_arg = DeclareLaunchArgument(
        'steering_mode',
        default_value='ackermann',
        description='Select steering mode: "ackermann" or "bicycle"'
    )
    # Declare launch argument for steering mode
    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='stanley',
        description='Select control mode: "pid" , "pure_pursuit" or "stanley"'
    )

    # Declare launch argument for lowe_ratio
    lowe_ratio_arg = DeclareLaunchArgument(
        'lowe_ratio',
        default_value='0.3',
        description='Ratio of distance for best matching in lowe ratio'
    )

    # Declare launch argument for save data
    save_data_arg = DeclareLaunchArgument(
        'save_data',
        default_value='false',
        description='Set to true to save data for visual odometry'
    )

    # Declare launch argument for file name
    filename_arg = DeclareLaunchArgument(
        'filename',
        default_value='visual_odometry_data',
        description='File name to save visual odometry data'
    )

    # launch sim gazebo
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    pkg_rover_gazebo,
                    "launch",
                    "sim.launch.py"
                )
            ]
        )
    )

    # Steering model node with configurable steering mode
    steering_model_node = Node(
        package="rover_controller",
        executable="steering_model_node.py",
        name="steering_model_node",
        parameters=[{"steering_mode": LaunchConfiguration("steering_mode")}]
    )

    odometry_calculation = Node(
        package="rover_controller",
        executable="odometry_calculation.py",
        name="odometry_calculation"
    )

    launch_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        namespace='',
        output='screen',
        prefix='xterm -e')
    
    path_publisher = Node(
        package="rover_controller",
        executable="path_publisher.py",
        name="path_publisher"
    )

    controller_server = Node(
        package="rover_controller",
        executable="controller_server.py",
        name="controller_server",
        parameters=[{"control_mode": LaunchConfiguration("control_mode")},
                    {"save_data": LaunchConfiguration("save_data")},
                    {"filename": LaunchConfiguration("filename")}
                    ]
    )

    visual_odom_orb_bf = Node(
        package="rover_controller",
        executable="visual_odom_orb_bf.py",
        name="visual_odometry",
        parameters=[{"lowe_ratio": LaunchConfiguration("lowe_ratio")}]
    )

    launch_description = LaunchDescription()
    launch_description.add_action(steering_mode_arg)
    launch_description.add_action(control_mode_arg)
    launch_description.add_action(lowe_ratio_arg)
    launch_description.add_action(sim)
    launch_description.add_action(steering_model_node)
    launch_description.add_action(odometry_calculation)
    launch_description.add_action(launch_teleop)
    launch_description.add_action(path_publisher)
    
    return launch_description