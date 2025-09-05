import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params = os.path.join(get_package_share_directory('mobile_robot'), 'config', 'rtabmap.yaml')
    rtab = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[params],
        remappings=[('scan', '/scan')],
    )
    return LaunchDescription([rtab])

