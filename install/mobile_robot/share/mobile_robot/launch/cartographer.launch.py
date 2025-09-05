import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    cfg_dir  = os.path.join(get_package_share_directory('mobile_robot'), 'config', 'cartographer')
    cfg_base = 'cartographer.lua' 

    carto = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-configuration_directory', cfg_dir,
                   '-configuration_basename', cfg_base],
        remappings=[('scan', '/scan')],
    )

    grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',  # <— correct name on Humble
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': True, 'resolution': 0.05, 'publish_period_sec': 1.0}],
    )
    return LaunchDescription([carto, grid])

