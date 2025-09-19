import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    This is the default launch file for the 'pioneer_gazebo' package.
    It is launched when running 'framework launch --all'.
    By default, it includes the package-specific launch file.
    """
    pkg_specific_launch_file_path = os.path.join(
        get_package_share_directory('pioneer_gazebo'),
        'launch',
        'pioneer_gazebo_launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(pkg_specific_launch_file_path))
    ])
