import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('robot_localization_package')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')

    my_robot_driver = WebotsController(
        robot_name='my_robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )
    webots= WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'epuck_world.wbt'),
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[{"yaml_filename": os.path.join(package_dir, "resource", "my_map.yaml")}],
        output="screen"
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": ["map_server"]
        }]
    )

    tf_static = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_broadcaster",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(package_dir, "rviz", "pf.rviz")],
        output="screen"
    )

    fake_feature_extractor = Node(
            package="robot_localization_package",
            executable="fake_feature_extractor",
            name="fake_feature_extractor",
            output="screen"
    )

    particle_filter = Node(
            package="robot_localization_package",
            executable="particle_filter",
            name="particle_filter",
            output="screen"
    )


    return LaunchDescription([
        rviz,
        webots,
        my_robot_driver,
        fake_feature_extractor,
        particle_filter,
        tf_static,
        map_server,
        lifecycle_manager
    ])