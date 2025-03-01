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
        launch_arguments={'world': os.path.join(epuck_share, 'worlds', 'epuck_world.wbt')}.items()
    )

    ## ✅ Corrected: Removed `robot_state_publisher`
    webots_driver_node = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        parameters=[{'robot_description': os.path.join(epuck_share, 'resource', 'epuck_webots.urdf')}]
    )
#
    #feature_extractor_node = Node(
    #    package='robot_localization_package',
    #    executable='feature_extractor',
    #    output='screen'
    #)
#
    #particle_filter_node = Node(
    #    package='robot_localization_package',
    #    executable='particle_filter',
    #    output='screen'
    #)
#
    #map_server_node = Node(
    #    package='nav2_map_server',
    #    executable='map_server',
    #    name='map_server',
    #    output='screen',
    #    parameters=[{'yaml_filename': os.path.join(local_share, 'resource', 'my_map.yaml')}]
    #)
#
    #rviz_config_file = os.path.join(local_share, 'resource', 'rviz_config.rviz')
    #if not os.path.exists(rviz_config_file):
    #    print("⚠️ Warning: RViz config file not found. Defaulting to empty visualization.")
#
    #rviz_node = Node(
    #    package='rviz2',
    #    executable='rviz2',
    #    arguments=['--display-config', rviz_config_file],
    #    output='screen'
    #)
#
    #teleop_node = Node(
    #    package='teleop_twist_keyboard',
    #    executable='teleop_twist_keyboard',
    #    output='screen',
    #    prefix='xterm -e',
    #    remappings=[('/cmd_vel', '/epuck/cmd_vel')]
    #)
#
    ## ----------------- ADD STATIC TF PUBLISHERS ----------------- #
    #static_tf_publishers = [
    #    Node(
    #        package="tf2_ros",
    #        executable="static_transform_publisher",
    #        arguments=[str(x), str(y), "0", "0", "0", "0", "map", f"corner_{i+1}"]
    #    )
    #    for i, (x, y) in enumerate([
    #        (0.75, 0.75),    # Top-right
    #        (-0.75, 0.75),   # Top-left
    #        (-0.75, -0.75),  # Bottom-left
    #        (0.75, -0.75)    # Bottom-right
    #    ])
    #]
#
    #wooden_box_tf_publishers = [
    #    Node(
    #        package="tf2_ros",
    #        executable="static_transform_publisher",
    #        arguments=[str(x), str(y), "0", "0", "0", "0", "map", f"wooden_box_{i+1}"]
    #    )
    #    for i, (x, y) in enumerate([
    #        (-0.265062, 0.13),
    #        (-0.115895, -0.36),
    #        (0.44, 0.12),
    #        (0.29726, -0.29),
    #        (-0.158467, 0.26)
    #    ])
    #]
    # ------------------------------------------------------------ #

    return LaunchDescription([
        webots_launch,
        webots_driver_node,  # ✅ Now Webots handles TF publishing
        #feature_extractor_node,
        #particle_filter_node,
        #map_server_node,
        #rviz_node,
        #teleop_node
    ]) #+ static_tf_publishers + wooden_box_tf_publishers)  # Append TF publishers
