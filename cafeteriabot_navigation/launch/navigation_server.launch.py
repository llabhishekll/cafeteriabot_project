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
    display_rviz = LaunchConfiguration("display_rviz", default="True")

    # package cafeteriabot_navigation
    path_root = Path(get_package_share_directory("cafeteriabot_navigation"))
    path_rviz = path_root / "rviz" / "navigation_server.config.rviz"

    # maps file path
    path_maps = path_root / "maps"

    # return launch
    return LaunchDescription(
        [
            DeclareLaunchArgument(name="use_sim_time", default_value="True"),
            DeclareLaunchArgument(name="display_rviz", default_value="True"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher",
                namespace="",
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", "robot_base_link", "base_link"],
                condition=IfCondition(LaunchConfiguration('use_sim_time')),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher",
                namespace="",
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", "cleaner_2/base_link", "base_link"],
                condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
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
                                "cafeteria_map_sim.yaml"
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
                                "cafeteria_map_real.yaml"
                            ]
                        ),
                    }
                ],
                condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
            ),
            Node(
                package="nav2_map_server",
                executable="costmap_filter_info_server",
                name="costmap_filter_info_node",
                output="screen",
                parameters=[
                    generate_yaml_param(
                        path_root / "config" / "sim" / "filter_mask.config.yml",
                        {
                            "yaml_filename": 
                            (path_maps / "cafeteria_map_mask_sim.yaml").as_posix(),
                        }
                    ),
                ],
                condition=IfCondition(LaunchConfiguration('use_sim_time')),
            ),
            Node(
                package="nav2_map_server",
                executable="costmap_filter_info_server",
                name="costmap_filter_info_node",
                output="screen",
                parameters=[
                    generate_yaml_param(
                        path_root / "config" / "real" / "filter_mask.config.yml",
                        {
                            "yaml_filename": 
                            (path_maps / "cafeteria_map_mask_real.yaml").as_posix(),
                        }
                    ),
                ],
                condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="filter_mask_node",
                output="screen",
                parameters=[
                    generate_yaml_param(
                        path_root / "config" / "sim" / "filter_mask.config.yml",
                        {
                            "yaml_filename": 
                            (path_maps / "cafeteria_map_mask_sim.yaml").as_posix(),
                        }
                    ),
                ],
                condition=IfCondition(LaunchConfiguration('use_sim_time')),
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="filter_mask_node",
                output="screen",
                parameters=[
                    generate_yaml_param(
                        path_root / "config" / "real" / "filter_mask.config.yml",
                        {
                            "yaml_filename": 
                            (path_maps / "cafeteria_map_mask_real.yaml").as_posix(),
                        }
                    ),
                ],
                condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="localization_node",
                output="screen",
                parameters=[
                    generate_yaml_param(
                        path_root / "config" / "sim" / "localization_server.config.yml",
                    ),
                ],
                condition=IfCondition(LaunchConfiguration('use_sim_time')),
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="localization_node",
                output="screen",
                remappings=[
                    ("/scan", "/cleaner_2/scan"),
                ],
                parameters=[
                    generate_yaml_param(
                        path_root / "config" / "real" / "localization_server.config.yml",
                    ),
                ],
                condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server_node",
                output="screen",
                parameters=[
                    generate_yaml_param(
                        path_root / "config" / "sim" / "planner_server.config.yml",
                    ),
                ],
                condition=IfCondition(LaunchConfiguration('use_sim_time')),
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server_node",
                output="screen",
                remappings=[
                    ("/scan", "/cleaner_2/scan"),
                ],
                parameters=[
                    generate_yaml_param(
                        path_root / "config" / "real" / "planner_server.config.yml",
                    ),
                ],
                condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server_node",
                output="screen",
                remappings=[
                    ("/cmd_vel", "diffbot_base_controller/cmd_vel_unstamped"),
                ],
                parameters=[
                    generate_yaml_param(
                        path_root / "config" / "sim" / "controller_server.config.yml",
                    ),
                ],
                condition=IfCondition(LaunchConfiguration('use_sim_time')),
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server_node",
                output="screen",
                remappings=[
                    ("/odom", "/cleaner_2/odom"),
                    ("/cmd_vel", "/cleaner_2/cmd_vel"),
                ],
                parameters=[
                    generate_yaml_param(
                        path_root / "config" / "real" / "controller_server.config.yml",
                    ),
                ],
                condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server_node",
                output="screen",
                parameters=[
                    generate_yaml_param(
                        path_root / "config" / "sim" / "behavior_server.config.yml",
                    ),
                ],
                condition=IfCondition(LaunchConfiguration('use_sim_time')),
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server_node",
                output="screen",
                parameters=[
                    generate_yaml_param(
                        path_root / "config" / "real" / "behavior_server.config.yml",
                    ),
                ],
                condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator_node",
                output="screen",
                parameters=[
                    generate_yaml_param(
                        path_root / "config" / "sim" / "behavior_tree.config.yml",
                        {
                            "default_nav_to_pose_bt_xml": 
                            (path_root / "config" / "behavior_tree.xml").as_posix(),
                            "default_nav_through_poses_bt_xml":
                            (path_root / "config" / "behavior_tree.xml").as_posix(),
                        }
                    ),
                ],
                condition=IfCondition(LaunchConfiguration('use_sim_time')),
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator_node",
                output="screen",
                remappings=[
                    ("/odom", "/cleaner_2/odom"),
                ],
                parameters=[
                    generate_yaml_param(
                        path_root / "config" / "real" / "behavior_tree.config.yml",
                        {
                            "default_nav_to_pose_bt_xml": 
                            (path_root / "config" / "behavior_tree.xml").as_posix(),
                             "default_nav_through_poses_bt_xml":
                            (path_root / "config" / "behavior_tree.xml").as_posix(),
                        }
                    ),
                ],
                condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
            ),
            TimerAction(
                period=8.0,
                actions=[
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="localization_manager_node",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "autostart": True,
                                "node_names": [
                                    "map_server_node",
                                    "costmap_filter_info_node",
                                    "filter_mask_node",
                                    "localization_node",
                                ]
                            }
                        ],
                    ),
                ],
            ),
            TimerAction(
                period=8.0,
                actions=[
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="navigation_manager_node",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "autostart": True,
                                "node_names": [
                                    "planner_server_node",
                                    "controller_server_node",
                                    "behavior_server_node",
                                    "bt_navigator_node",
                                ]
                            }
                        ],
                    ),
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
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
