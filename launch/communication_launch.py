from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config=get_package_share_directory('rslidar_sdk')+'/rviz/rviz2.rviz'

    return LaunchDescription([
        Node(package='communication', executable='communication', output='screen'),
        Node(package='communication', executable='LiDAR', output='screen'),     
        Node(package='ros2_whill', executable='whill_modelc_controller', output='screen'),   
        Node(namespace='rslidar_sdk', package='rslidar_sdk', executable='rslidar_sdk_node', output='screen'),
    ])
