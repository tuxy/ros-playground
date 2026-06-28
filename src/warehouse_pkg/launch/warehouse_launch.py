from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
            # Fleet management node (just hashes the serial and returns a bay number)
            Node(
                package='warehouse_pkg',
                executable='fleet_manager',
                name='fleet_manager_server',
            ),
            
            # Robot nodes
            Node(
                package='warehouse_pkg',
                executable='robot_navigator',
                namespace='robot_048333c8',
                parameters=[{'serial': '048333c8'}],
            ),
            Node(
                package='warehouse_pkg',
                executable='robot_navigator',
                namespace='robot_6cafe0be',
                parameters=[{'serial': '6cafe0be'}]
            ),
            Node(
                package='warehouse_pkg',
                executable='robot_navigator',
                namespace='robot_0f4b05d7',
                parameters=[{'serial': '0f4b05d7'}]
            ),

            # Telemetry node; define assigned namespaces here (doesn't work for now)
            Node(
                package='warehouse_pkg',
                executable='telemetry_monitor',
                name='telemetry_monitor_node',
                output='screen'
            )
    ])
