import launch
import launch_ros
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  apm_config = PathJoinSubstitution([
    FindPackageShare('mavros'),
    'launch',
    'apm_config.yaml'
  ])

  pluginlists_yaml = PathJoinSubstitution([
    FindPackageShare('bluerov_ros2'),
    'launch',
    'mav_pluginlists.yaml'
  ])

  user_node = launch_ros.actions.Node(
      package='bluerov_ros2',
      executable='user.py',
      output='screen'
  )

  mavros_node = launch_ros.actions.Node(
    package='mavros',
    executable='mavros_node',
    output='screen',
    parameters=[{
      'fcu_url': LaunchConfiguration('fcu_url'),
      'gcs_url': LaunchConfiguration('gcs_url'),
      'system_id': LaunchConfiguration('system_id'),
      'component_id': LaunchConfiguration('component_id'),
      'target_system_id': LaunchConfiguration('tgt_system'),
      'target_component_id': LaunchConfiguration('tgt_component')
    },
    apm_config,
    pluginlists_yaml]
  )

  # joystick_node = launch_ros.actions.Node(
  #     package="joy",
  #     executable="joy_node",
  #     parameters=[{
  #         'dev': LaunchConfiguration('joy_dev'),
  #     }],
  #     output='screen'
  # )
  
  return launch.LaunchDescription([
    DeclareLaunchArgument(name="fcu_url", default_value="udp://:14551@127.0.0.1:14549"),
    DeclareLaunchArgument(name="gcs_url", default_value="udp://:14549@127.0.0.1:14548"),
    DeclareLaunchArgument(name="system_id", default_value="255"),
    DeclareLaunchArgument(name="component_id", default_value="240"),
    DeclareLaunchArgument(name="tgt_system", default_value="1"),
    DeclareLaunchArgument(name="tgt_component", default_value="1"),
    DeclareLaunchArgument(name="joy_dev", default_value="/dev/input/js0"),
    DeclareLaunchArgument(name="log_output", default_value="screen"),
    # user_node,
    mavros_node,
    user_node
    # joystick_node
  ])
