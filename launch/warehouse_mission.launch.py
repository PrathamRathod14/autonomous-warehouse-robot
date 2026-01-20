from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="autonomous_warehouse_mission",
            executable="warehouse_delivery",
            name="warehouse_delivery_mission",
            output="screen",
            parameters=[
                {"warehouse_yaml": LaunchConfiguration("warehouse_yaml")},
                {"csv_log_path": LaunchConfiguration("csv_log_path")},
            ],
        ),
    ])
