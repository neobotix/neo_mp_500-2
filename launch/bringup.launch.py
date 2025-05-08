# Neobotix GmbH
# Author: Pradheep Padmanabhan
# Contributor: Adarsh Karan K P

import launch
import xacro
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument,
  IncludeLaunchDescription,
  OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

from launch.launch_context import LaunchContext
from launch.conditions import IfCondition

def execution_stage(context: LaunchContext,
                    robot_namespace,
                    imu_enable,
                    uss_enable,
                    scanner_type):

    imu_enabl = str(imu_enable.perform(context))
    uss_enabl = str(uss_enable.perform(context))
    scanner_typ = str(scanner_type.perform(context))
    neo_mp_500 = get_package_share_directory('neo_mp_500-2')

    rp_ns = ""
    if (robot_namespace.perform(context) != "/"):
        rp_ns = robot_namespace.perform(context) + "/"

    launch_actions = []

    # Setting up the URDF
    urdf = os.path.join(neo_mp_500,
        'robot_model',
        'mp_500.urdf.xacro')

    # Start robot state publisher
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        namespace=robot_namespace,
        parameters=[{
            'robot_description': Command([
                "xacro", " ", urdf,
                " ", 'use_imu:=', imu_enabl,
                " ", 'use_uss:=', uss_enabl,
                " ", 'scanner_type:=', scanner_typ,
            ]),
            'frame_prefix': rp_ns
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ],
        arguments=[urdf]
    )

    launch_actions.append(start_robot_state_publisher_cmd)

    #  Launch hardware nodes
    # 1. Relayboard
    relayboard = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(neo_mp_500, 'configs/relayboard_v2', 'relayboard_v2.launch.py')
            ),
            launch_arguments={
                'namespace': robot_namespace
            }.items()
        )

    launch_actions.append(relayboard)

    # 2. Kinematics
    kinematics = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(neo_mp_500, 'configs/kinematics', 'kinematics.launch.py')
            ),
            launch_arguments={
                'namespace': robot_namespace
            }.items()
        )

    launch_actions.append(kinematics)

    # 3. Teleop
    teleop = IncludeLaunchDescription(
             PythonLaunchDescriptionSource(
                 os.path.join(neo_mp_500, 'configs/teleop', 'teleop.launch.py')
            ),
            launch_arguments={
                'namespace': robot_namespace
            }.items()
        )

    launch_actions.append(teleop)

    # 4. Laser
    laser = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(neo_mp_500, 'configs/lidar/sick/s300', f'{scanner_typ}.launch.py')
            ),
            launch_arguments={
                'namespace': robot_namespace
            }.items()
        )
    launch_actions.append(laser)

    # 5. IMU
    imu = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(neo_mp_500,
                    'configs/phidget_imu',
                    'imu_launch.py')
            ),
            launch_arguments={
                'namespace': robot_namespace
            }.items(),
            condition=IfCondition(imu_enable)
        )

    launch_actions.append(imu)

    return launch_actions

def generate_launch_description():

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
            'robot_namespace', default_value='', 
            description='Top-level namespace'
        )

    declare_imu_cmd = DeclareLaunchArgument(
            'imu_enable', default_value='False',
            description='Enable IMU - Options: True/False'
        )

    declare_uss_cmd = DeclareLaunchArgument(
            'uss_enable', default_value='False',
            description='Enable uss - Options: True/False'
        )

    declare_scanner_type_cmd = DeclareLaunchArgument(
            'scanner_type', default_value='sick_s300',
            choices=['', 'sick_s300', 'sick_microscan3'],
            description='Type of laser scanner to use'
        )

    # Opaque function for configuring URDF, IMU, Realsense and the USBoard
    opq_function = OpaqueFunction(
    function=execution_stage, 
    args=[
        LaunchConfiguration('robot_namespace'),
        LaunchConfiguration('imu_enable'),
        LaunchConfiguration('uss_enable'),
        LaunchConfiguration('scanner_type'),
        ])

    ld = LaunchDescription([
        declare_namespace_cmd,
        declare_imu_cmd,
        declare_uss_cmd,
        declare_scanner_type_cmd,
        opq_function
    ])
    return ld
