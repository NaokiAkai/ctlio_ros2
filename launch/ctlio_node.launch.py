from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  config_dir = os.path.join(
    get_package_share_directory('ctlio_ros2'),
    'config'
  )

  lio_config_yaml = os.path.join(config_dir, 'ctlio_config.yaml')

  return LaunchDescription([
    Node(
      package='ctlio_ros2',
      executable='ctlio_ros2_node',
      name='ctlio_ros2_node',
      parameters=[
        lio_config_yaml,
        {'param_files_dir': config_dir}
      ],
      output='screen'
    )
  ])
