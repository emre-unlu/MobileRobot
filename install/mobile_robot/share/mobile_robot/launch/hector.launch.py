import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params = os.path.join(get_package_share_directory('mobile_robot'), 'config', 'hector.yaml')
    hector = Node(
        package='hector_mapping',
        executable='hector_mapping',
        name='hector_mapping',
        output='screen',
        parameters=[params],
        remappings=[('scan', '/scan')],
    )
    return LaunchDescription([hector])

