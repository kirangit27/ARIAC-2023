import yaml
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    node = Node(
        package='ARIAC-2023',
        executable='rwa1',
        name='rwa1',
        #output='screen'
    )

    ld.add_action(node)

    return ld