from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from nav2_common.launch import RewrittenYaml


def generate_yaml_param(path, param_rewrites={}):
    # dynamic substitutions for parameters
    return RewrittenYaml(
        source_file=path.as_posix(),
        param_rewrites=param_rewrites,
    )


def generate_launch_description():
    # launch parameters
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    # package cafeteriabot_firmware
    path_root = Path(get_package_share_directory("cafeteriabot_firmware"))

    # return launch
    return LaunchDescription(
        [
            DeclareLaunchArgument(name="use_sim_time", default_value="True"),
            Node(
                package="cafeteriabot_firmware",
                executable="laser_scan_filtering",
                name="filtering01_node",
                output="screen",
                remappings=[
                    ("/map", "/keepout_filter_mask"),
                ],
                parameters=[
                    {
                        "distance_threshold": 2.2,
                        "f_ref": "map",
                        "t_ref": "robot_front_laser_base_link"
                    }
                ],
                condition=IfCondition(LaunchConfiguration("use_sim_time")),
            ),
            Node(
                package="cafeteriabot_firmware",
                executable="laser_scan_clustering",
                name="clustering_node",
                output="screen",
                parameters=[
                    {
                        "f_ref": "map",
                        "t_ref": "robot_front_laser_base_link",
                        "publish_rate": 1.0,
                        "distance_threshold": 0.1,
                        "min_points": 10,
                        "max_points": 120,
                        "stale_duration": 1,
                    }
                ],
                condition=IfCondition(LaunchConfiguration("use_sim_time")),
            ),
            Node(
                package="cafeteriabot_firmware",
                executable="laser_scan_detection",
                name="detection01_node",
                output="screen",
                parameters=[
                    {
                        "f_ref": "map",
                        "publish_rate": 1.0,
                        "min_distance": 0.45,
                        "max_distance": 0.75,
                        "angle_tolerance": 5,
                        "marker_duration": 5,
                        "stale_duration": 5,
                    }
                ],
                condition=IfCondition(LaunchConfiguration("use_sim_time")),
            ),
        ]
    )
