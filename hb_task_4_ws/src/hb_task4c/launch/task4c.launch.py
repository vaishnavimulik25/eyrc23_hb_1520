''' 
*****************************************************************************************
*
*        =============================================
*                  HB Theme (eYRC 2023-24)
*        =============================================
*
*
*  Filename:			Spawn_bot.launch.py
*  Description:         Use this file to spawn bot.
*  Created:				16/07/2023
*  Last Modified:	    16/09/2023
*  Modified by:         Srivenkateshwar
*  Author:				e-Yantra Team
*  
*****************************************************************************************
'''

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription ,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution,LaunchConfiguration, PythonExpression
import os
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, RegisterEventHandler, TimerAction)


def generate_launch_description():
    share_dir = get_package_share_directory('hb_task4c')
    
    camera = ExecuteProcess(                                                
    cmd=[[                                                                     
        'ros2 launch usb_cam camera.launch.py'
    ]],                                                                        
    shell=True
    )

    feedback_node = Node(
        package = "hb_task4c",
        executable = "feedback"
    )

    Controller_node1 = Node(
        package="hb_task4c",
        executable="controller1"
    )

    Controller_node2 = Node(
        package="hb_task4c",
        executable="controller2"
    )

    Controller_node3 = Node(
        package="hb_task4c",
        executable="controller3"
    )

    Next_Goal = Node(
        package="hb_task4c",
        executable="nextgoalpub"
    )
    return LaunchDescription([
        camera,
        feedback_node,
        Controller_node1,
        #Controller_node2,
        #Controller_node3,
        Next_Goal
        ])
