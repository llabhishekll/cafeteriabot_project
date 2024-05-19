import os
from glob import glob
from setuptools import setup

package_name = "cafeteriabot_firmware"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "point_marker_analysis = cafeteriabot_firmware.point_marker_analysis:main",
            "pose_marker_analysis = cafeteriabot_firmware.pose_marker_analysis:main",
            "filter_algorithm_analysis = cafeteriabot_firmware.filter_algorithm_analysis:main",
            "cluster_algorithm_analysis = cafeteriabot_firmware.cluster_algorithm_analysis:main",
            "laser_scan_filtering = cafeteriabot_firmware.laser_scan_filtering:main",
            "laser_scan_clustering = cafeteriabot_firmware.laser_scan_clustering:main",
            "laser_scan_detection = cafeteriabot_firmware.laser_scan_detection:main",
            "elevator_management = cafeteriabot_firmware.elevator_management:main",
            "table_docking_service = cafeteriabot_firmware.table_docking_service:main",
            "cafeteriabot_control = cafeteriabot_firmware.cafeteriabot_control:main",
            "pid_control_visualizer = cafeteriabot_firmware.pid_control_visualizer:main",
        ],
    },
)
