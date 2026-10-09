import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('volleyball_vision')
    config_file = os.path.join(pkg_share, 'config', 'vision_params.yaml')
    calib_file = os.path.join(pkg_share, 'config', 'stereo_calibration.yaml')

    node = Node(
        package='volleyball_vision',
        executable='volleyball_vision_node',
        name='volleyball_vision_node',
        output='screen',
        parameters=[
            config_file,
            {'calib_file': calib_file}
        ]
    )

    return LaunchDescription([node])
