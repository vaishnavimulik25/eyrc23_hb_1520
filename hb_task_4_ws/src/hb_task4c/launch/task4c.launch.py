from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription ,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution,LaunchConfiguration, PythonExpression
import os
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)

def generate_launch_description():


    Publisher_node = Node(
        package = "hb_task4c",
        executable = "publisher"
    )
    run_agent = ExecuteProcess(
    cmd=[[
        'ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888',

    ]],
    shell=True
)
    
    return LaunchDescription([
        Publisher_node,
        run_agent,
        RegisterEventHandler(
            OnProcessStart(
                target_action=Publisher_node,
                on_start=[
                    LogInfo(msg='Publishing info'),
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=run_agent,
                on_start=[
                    LogInfo(msg='run_agent started'),
                ]
            )
        ),
        RegisterEventHandler(
   	     OnShutdown(
        	on_shutdown=[LogInfo(
      	        	msg=['Launch was asked to shutdown: ',
           	     	LocalSubstitution('event.reason')]
                )]
    	     )
	),

        ])
