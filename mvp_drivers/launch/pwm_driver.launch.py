import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config_file_path = os.path.join(get_package_share_directory("mvp_drivers"), 'config/pwm.yaml')
    return LaunchDescription([
        Node(
            package='mvp_drivers',
            namespace='test_robot',
            executable='pwm_driver_node',
            name='pwm_driver_node',
            prefix=['stdbuf -o L'],
            output="screen",
            parameters=[
                        {'config_file_name': config_file_path},
                        ]
        )
       
    ])
