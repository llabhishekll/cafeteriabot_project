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
                executable="point_marker_analysis",
                name="point_marker_analysis_node",
                output="screen",
                parameters=[
                    {
                        "f_ref": "map",
                        "marker_duration": 5,
                    }
                ],
                condition=IfCondition(LaunchConfiguration("use_sim_time")),
            ),
            Node(
                package="cafeteriabot_firmware",
                executable="point_marker_analysis",
                name="point_marker_analysis_node",
                output="screen",
                parameters=[
                    {
                        "f_ref": "map",
                        "marker_duration": 5,
                    }
                ],
                condition=UnlessCondition(LaunchConfiguration("use_sim_time")),
            ),
            Node(
                package="cafeteriabot_firmware",
                executable="pose_marker_analysis",
                name="pose_marker_analysis_node",
                output="screen",
                parameters=[
                    {
                        "f_ref": "map",
                        "t_ref": "robot_front_laser_base_link",
                        "marker_duration": 5,
                    }
                ],
                condition=IfCondition(LaunchConfiguration("use_sim_time")),
            ),
            Node(
                package="cafeteriabot_firmware",
                executable="pose_marker_analysis",
                name="pose_marker_analysis_node",
                output="screen",
                parameters=[
                    {
                        "f_ref": "map",
                        "t_ref": "cleaner_2/base_link",
                        "marker_duration": 5,
                    }
                ],
                condition=UnlessCondition(LaunchConfiguration("use_sim_time")),
            ),
            Node(
                package="cafeteriabot_firmware",
                executable="filter_algorithm_analysis",
                name="filter_analysis_node",
                output="screen",
                remappings=[
                    ("/map", "/keepout_filter_mask"),
                ],
                parameters=[
                    {
                        "debug": True,
                        "distance_threshold": 2.2,
                        "f_ref": "map",
                        "t_ref": "robot_front_laser_base_link",
                        "marker_duration": 5,
                    }
                ],
                condition=IfCondition(LaunchConfiguration("use_sim_time")),
            ),
            Node(
                package="cafeteriabot_firmware",
                executable="filter_algorithm_analysis",
                name="filter_analysis_node",
                output="screen",
                remappings=[
                    ("/map", "/keepout_filter_mask"),
                    ("/scan", "/cleaner_2/scan"),
                ],
                parameters=[
                    {
                        "debug": True,
                        "distance_threshold": 2.2,
                        "f_ref": "map",
                        "t_ref": "cleaner_2/laser_sensor_link",
                        "marker_duration": 5,
                    }
                ],
                condition=UnlessCondition(LaunchConfiguration("use_sim_time")),
            ),
            Node(
                package="cafeteriabot_firmware",
                executable="laser_scan_filtering",
                name="filtering_node",
                output="screen",
                remappings=[
                    ("/map", "/keepout_filter_mask"),
                ],
                parameters=[
                    {
                        "f_ref": "map",
                        "t_ref": "robot_front_laser_base_link",
                        "distance_threshold": 2.2,
                    }
                ],
                condition=IfCondition(LaunchConfiguration("use_sim_time")),
            ),
            Node(
                package="cafeteriabot_firmware",
                executable="laser_scan_filtering",
                name="filtering_node",
                output="screen",
                remappings=[
                    ("/map", "/keepout_filter_mask"),
                    ("/scan", "/cleaner_2/scan"),
                ],
                parameters=[
                    {
                        "f_ref": "map",
                        "t_ref": "cleaner_2/laser_sensor_link",
                        "distance_threshold": 2.2,
                    }
                ],
                condition=UnlessCondition(LaunchConfiguration("use_sim_time")),
            ),
            Node(
                package="cafeteriabot_firmware",
                executable="cluster_algorithm_analysis",
                name="clustering_analysis_node",
                output="screen",
                parameters=[
                    {
                        "debug": True,
                        "f_ref": "map",
                        "t_ref": "robot_front_laser_base_link",
                        "distance_threshold": 0.1,
                        "min_points": 10,
                        "max_points": 120,
                        "marker_duration": 5,
                    }
                ],
                condition=IfCondition(LaunchConfiguration("use_sim_time")),
            ),
            Node(
                package="cafeteriabot_firmware",
                executable="cluster_algorithm_analysis",
                name="clustering_analysis_node",
                output="screen",
                parameters=[
                    {
                        "debug": True,
                        "f_ref": "map",
                        "t_ref": "cleaner_2/laser_sensor_link",
                        "distance_threshold": 0.1,
                        "min_points": 10,
                        "max_points": 120,
                        "marker_duration": 5,
                    }
                ],
                condition=UnlessCondition(LaunchConfiguration("use_sim_time")),
            ),
        ]
    )
