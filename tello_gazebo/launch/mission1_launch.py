import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_ros_pkg = get_package_share_directory("gazebo_ros")
    simu_pkg = get_package_share_directory("tello_gazebo")
    urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello_1.urdf')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, "launch", "gazebo.launch.py")
        )
    )
    os.environ["GAZEBO_MODEL_PATH"] += os.path.join(simu_pkg, "models")

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value = os.path.join(simu_pkg, "worlds", "aruco.world"),
            description = "World file to use for the simulation"
        ),
        DeclareLaunchArgument(
            "verbose",
            default_value = "true",
            description = "Increase messages written to terminal"
        ),

        gazebo_launch,

        #Spawn tello.urdf
        TimerAction(
           period = 1.0,
           actions = [
                Node(
                     package = "gazebo_ros",
                     executable = "spawn_entity.py",
                     arguments = [
                          "-entity", "drone_ipsa",
                          "-topic", "/robot_description",
                          "-x", "0.0"
                     ]
                )
           ]
        ),

        # Publish static transforms
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             arguments=[urdf_path])
    ])
