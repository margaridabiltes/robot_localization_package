from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
import os


def generate_launch_description():
    epuck_share = get_package_share_directory('webots_ros2_epuck')

    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(epuck_share, 'launch', 'robot_launch.py')
        ),
        launch_arguments={'world': os.path.join(epuck_share, 'worlds', 'epuck_world.wbt')}.items()
    )