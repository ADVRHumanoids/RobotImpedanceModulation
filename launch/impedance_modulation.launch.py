import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  config = os.path.join(
      get_package_share_directory('rim'),
      'config',
      'impedance_modulation_settings.yaml'
      )
  return LaunchDescription([
      Node(
            package='rim',
            executable='ImpedanceModulation',
            namespace='robot',
            name='impedance_settings',
            parameters=[config],
            output='screen',
            emulate_tty=True
          )
    ])