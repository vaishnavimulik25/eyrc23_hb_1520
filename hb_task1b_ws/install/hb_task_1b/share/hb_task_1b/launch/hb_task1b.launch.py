from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    service_node = Node(
        package="hb_task_1b",           # Enter the name of your ROS2 package
        executable="service_node",    # Enter the name of your executable
    )
    controller_node = Node(
        package="hb_task_1b",           # Enter the name of your ROS2 package
        executable="hb_task1b_controller",    # Enter the name of your executable
    )
    ld.add_action(controller_node)
    ld.add_action(service_node)

    return ld
