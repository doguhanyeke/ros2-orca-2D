from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    drone_id = LaunchConfiguration('drone_id', default='0')
    number_of_drones = LaunchConfiguration('number_of_drones', default='3')
    
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value='1',
        description='ID of the drone'
    )

    config = os.path.join(
        get_package_share_directory('ros2-orca'),
        'config',
        'params_ro_col.yaml'
    )
    
    ros2_orca_node = Node(
        package='ros2-orca',
        executable='ros2-orca',
        name='ros2_orca',
        parameters=[
            {'drone_id': drone_id},
            config
        ]
    )

    return LaunchDescription([
        drone_id_arg,
        ros2_orca_node,
    ])
