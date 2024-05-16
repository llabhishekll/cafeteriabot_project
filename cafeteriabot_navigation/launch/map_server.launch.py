from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition


# Note: this launch file is only for testing the map quality
def generate_launch_description():
    # launch parameters
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    display_rviz = LaunchConfiguration("display_rviz", default="True")
    map_file = LaunchConfiguration("map_file", default="")

    # package cafeteriabot_navigation
    path_root = Path(get_package_share_directory("cafeteriabot_navigation"))
    path_rviz = path_root / "rviz" / "map_server.config.rviz"

    # maps file path
    path_maps = path_root / "maps"

    # return launch
    return LaunchDescription(
        [
            DeclareLaunchArgument(name="use_sim_time", default_value="True"),
            DeclareLaunchArgument(name="display_rviz", default_value="True"),
            DeclareLaunchArgument(name="map_file", default_value=""),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher",
                namespace="",
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "yaml_filename": PathJoinSubstitution(
                            [
                                path_maps.as_posix(),
                                map_file
                            ]
                        ),
                    }
                ],
                condition=IfCondition(LaunchConfiguration('use_sim_time')),
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "yaml_filename": PathJoinSubstitution(
                            [
                                path_maps.as_posix(),
                                map_file
                            ]
                        ),
                    }
                ],
                condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="map_manager_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": [
                            "map_server_node",
                        ]
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time
                    }
                ],
                arguments=[
                    "-d", path_rviz.as_posix(),
                    "--ros-args", "--log-level", "WARN",
                ],
                condition=IfCondition(LaunchConfiguration('display_rviz'))
            ),
        ]
    )
