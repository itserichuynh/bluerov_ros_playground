import launch
import launch_ros
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    bluerov_node = launch_ros.actions.Node(
        package='bluerov_ros2',
        executable='bluerov_node.py',
        output='screen',
        parameters=[{'device': 'udpin:0.0.0.0:14550'}]
    )
    
    return launch.LaunchDescription([
        bluerov_node
    ])
