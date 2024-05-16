from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

# Note: this launch file is only for testing the localization quality
def generate_launch_description():
    # launch parameters
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    display_rviz = LaunchConfiguration("display_rviz", default="True")
    map_file = LaunchConfiguration("map_file", default="")

    # package cafeteriabot_navigation
    path_root = Path(get_package_share_directory("cafeteriabot_navigation"))
    path_rviz = path_root / "rviz" / "localization_server.config.rviz"

    # maps file path
    path_maps = path_root / "maps"

    # return launch
    return LaunchDescription(
        [
            DeclareLaunchArgument(name="use_sim_time", default_value="True"),
            DeclareLaunchArgument(name="display_rviz", default_value="True"),
            DeclareLaunchArgument(name="map_file", default_value=""),
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
                                map_file,
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
                                map_file,
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
                parameters=[
                    generate_yaml_param(
                        path_root / "config" / "real" / "localization_server.config.yml",
                    ),
                ],
                condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
            ),
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
