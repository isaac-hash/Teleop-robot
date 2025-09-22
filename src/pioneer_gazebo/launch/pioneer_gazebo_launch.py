from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Path to Gazebo world (your custom empty world in ~/robot/sim/worlds)
    world_file = os.path.expanduser("~/robot/sim/worlds/empty.world")

    # Path to Pioneer2DX model SDF
    pioneer_sdf = os.path.expanduser("~/robot/sim/models/Pioneer2DX/model.sdf")

    return LaunchDescription([
        # Start Gazebo (server + client) with the chosen world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ]),
            launch_arguments={'world': world_file}.items()
        ),

        # Spawn the Pioneer2DX directly from SDF
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', pioneer_sdf, '-entity', 'pioneer2dx'],
            output='screen'
        ),

        # Example control/demo node (if you have one)
        Node(
            package='pioneer_gazebo',
            executable='demo',
            name='demo',
            output='screen',
            emulate_tty=True
        ),
    ])
