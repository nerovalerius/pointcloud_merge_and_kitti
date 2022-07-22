from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_kitti_and_merge',
            namespace='laser_assembler_1',
            executable='laser_scan_assembler',
            name='my_assembler',
            parameters=[
              {"max_scans": "400"},
              {"fixed_frame": "livox_frame"}
            ],
            remappings=[
                ('/scan', '/laser_scan'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        ),
        Node(
            package='pointcloud_kitti_and_merge',
            namespace='merge_1',
            executable='merge',
            name='merge',
        )
    ])