from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
 # Path to Gazebo world
    world_file = "/usr/share/gazebo-11/worlds/empty.world"

    # Path to Pioneer2DX model SDF
    pioneer_sdf = os.path.expanduser("~/.gazebo/models/model.sdf")
    return LaunchDescription([
    # Start Gazebo (server + client)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ]),
            launch_arguments={'world': world_file}.items()
        ),

        # Spawn the Pioneer2DX into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', pioneer_sdf, '-entity', 'pioneer2dx'],
            output='screen'
        ),
        
        Node(
            package='pioneer_gazebo',
            executable='demo',
            name='demo',
            output='screen',
            emulate_tty=True
        ),
    ])
