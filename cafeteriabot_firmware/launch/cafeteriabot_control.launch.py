from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition


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
                executable="table_docking_service",
                name="table_docking_service_node",
                output="screen",
                remappings=[
                    ("/cmd_vel", "/diffbot_base_controller/cmd_vel_unstamped"),
                ],
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "f_ref": "map",
                        "t_ref": "robot_base_footprint",
                        "retry_duration": 5,
                        "distance_away": 0.65,
                        "step_size": 0.5,
                        "node_rate": 10.0,
                        "goal_tolerance": 0.15,
                        "yaw_tolerance": 0.15,
                    }
                ],
                condition=IfCondition(LaunchConfiguration("use_sim_time")),
            ),
            Node(
                package="cafeteriabot_firmware",
                executable="table_docking_service",
                name="table_docking_service_node",
                output="screen",
                remappings=[
                    ("/cmd_vel", "/cleaner_2/cmd_vel"),
                ],
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "f_ref": "map",
                        "t_ref": "cleaner_2/base_link",
                        "retry_duration": 5,
                        "distance_away": 0.5,
                        "step_size": 0.5,
                        "node_rate": 10.0,
                        "goal_tolerance": 0.05,
                        "yaw_tolerance": 0.05,
                    }
                ],
                condition=UnlessCondition(LaunchConfiguration("use_sim_time")),
            ),
            Node(
                package="cafeteriabot_firmware",
                executable="elevator_management",
                name="elevator_management_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "robot_radius": 0.22,
                        "table_radius": 0.35
                    }
                ],
                condition=IfCondition(LaunchConfiguration("use_sim_time")),
            ),
            Node(
                package="cafeteriabot_firmware",
                executable="elevator_management",
                name="elevator_management_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "robot_radius": 0.22,
                        "table_radius": 0.35
                    }
                ],
                condition=UnlessCondition(LaunchConfiguration("use_sim_time")),
            ),
            Node(
                package="cafeteriabot_firmware",
                executable="cafeteriabot_control",
                name="cafeteria_robot_control_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "f_ref": "map",
                        "t_ref": "robot_base_footprint",
                        "yaml_filename": PathJoinSubstitution(
                            [
                                (path_root / "config").as_posix(),
                                "waypoints.yaml"
                            ]
                        ),
                        "retry_duration": 5,
                        "node_rate": 10.0,
                        "marker_duration": 0,
                        "cancel_duration": 5,
                    }
                ],
                condition=IfCondition(LaunchConfiguration("use_sim_time")),
            ),
            Node(
                package="cafeteriabot_firmware",
                executable="cafeteriabot_control",
                name="cafeteria_robot_control_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "f_ref": "map",
                        "t_ref": "cleaner_2/base_link",
                        "yaml_filename": PathJoinSubstitution(
                            [
                                (path_root / "config").as_posix(),
                                "waypoints.yaml"
                            ]
                        ),
                        "retry_duration": 5,
                        "node_rate": 10.0,
                        "marker_duration": 0,
                        "cancel_duration": 5,
                    }
                ],
                condition=UnlessCondition(LaunchConfiguration("use_sim_time")),
            ),
        ]
    )
