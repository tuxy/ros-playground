from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    fleet_manager = Node(
        package='warehouse_pkg',
        executable='fleet_manager',
        name='fleet_manager_server',
    )

    robot_serials = ['robot_048333c8', 'robot_6cafe0be', 'robot_0f4b05d7']
    robot_nodes = [
        Node(
            package='warehouse_pkg',
            executable='robot_navigator',
            namespace=serial,
            parameters=[{'serial': serial}],
        ) for serial in robot_serials
    ]

    launch_items = robot_nodes + [fleet_manager]
    return LaunchDescription(launch_items)
