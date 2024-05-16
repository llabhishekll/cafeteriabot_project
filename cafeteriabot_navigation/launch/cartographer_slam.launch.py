from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # launch parameters
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    # package cafeteriabot_navigation
    path_root = Path(get_package_share_directory("cafeteriabot_navigation"))
    path_rviz = path_root / "rviz" / "cartographer_slam.config.rviz"
    path_conf = path_root / "config"

    # return launch
    return LaunchDescription(
        [
            DeclareLaunchArgument(name="use_sim_time", default_value="True"),
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                    }
                ],
                arguments=[
                    "-configuration_directory", (path_conf / "sim").as_posix(),
                    "-configuration_basename", "cartographer.config.lua",
                ],
                condition=IfCondition(LaunchConfiguration("use_sim_time")),
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node",
                output="screen",
                remappings=[
                    ("/tf", "/cleaner_2/tf"),
                    ("/odom", "/cleaner_2/odom"),
                    ("/scan", "/cleaner_2/scan"),
                ],
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                    }
                ],
                arguments=[
                    "-configuration_directory", (path_conf / "real").as_posix(),
                    "-configuration_basename", "cartographer.config.lua",
                ],
                condition=UnlessCondition(LaunchConfiguration("use_sim_time")),
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                name="cartographer_occupancy_grid_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                    }
                ],
                arguments=[
                    "-resolution", "0.01",
                    "-publish_period_sec", "1.0",
                ],
                condition=IfCondition(LaunchConfiguration("use_sim_time")),
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                name="cartographer_occupancy_grid_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                    }
                ],
                remappings=[
                    ("/tf", "/cleaner_2/tf"),
                    ("/odom", "/cleaner_2/odom"),
                    ("/scan", "/cleaner_2/scan"),
                ],
                arguments=[
                    "-resolution", "0.01",
                    "-publish_period_sec", "1.0",
                ],
                condition=UnlessCondition(LaunchConfiguration("use_sim_time")),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher",
                namespace="",
                output="screen",
                arguments=[
                    "0", "0", "0.557", "0", "-0.027", "0", "base_footprint", "laser_sensor_link"
                ],
                condition=UnlessCondition(LaunchConfiguration("use_sim_time")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=[
                    "-d", path_rviz.as_posix(),
                ],
            ),
            LogInfo(
                msg="To save map file run `ros2 run nav2_map_server map_saver_cli -f <file_name>`"
            ),
        ]
    )