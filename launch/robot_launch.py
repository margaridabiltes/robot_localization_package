import os
import launch

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    epuck_share = get_package_share_directory('webots_ros2_epuck')
    local_share = get_package_share_directory('robot_localization_package')

    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(epuck_share, 'launch', 'robot_launch.py')
        ),
        launch_arguments={'world': os.path.join(epuck_share, 'worlds', 'epuck_world.wbt')}.items()  # Use world from epuck simulation
    )

    urdf_file = os.path.join(epuck_share, 'resource', 'epuck_webots.urdf')
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )

     # estimates pose in `map` frame
    particle_filter_node = Node(
        package='robot_localization_package',
        executable='particle_filter',
        output='screen'
    )

    
    # ir buscar do webots_ros2 também 
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': os.path.join(local_share, 'resource', 'my_map.yaml')}]
    )

    # Particle Filter Node
    particle_filter_node = Node(
        package='robot_localization_package',
        executable='particle_filter',
        output='screen'
    )

    # ver onde ir buscar isto tbm
    rviz_config_file = os.path.join(local_share, 'resource', 'rviz_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['--display-config', rviz_config_file],
        output='screen'
    )

    # Teleop Node
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        remappings=[('/cmd_vel', '/epuck/cmd_vel')]  # Ensure it maps to the e-puck's expected topic
    )

    return LaunchDescription([
        webots_launch,
        rsp_node,
        map_server_node,
        particle_filter_node,
        rviz_node,
        teleop_node
    ])
