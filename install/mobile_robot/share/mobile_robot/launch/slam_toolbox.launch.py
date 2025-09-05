import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params = os.path.join(get_package_share_directory('mobile_robot'), 'config', 'slam.yaml')
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py')
        ),
        launch_arguments={'slam_params_file': params}.items(),
    )
    return LaunchDescription([slam])

