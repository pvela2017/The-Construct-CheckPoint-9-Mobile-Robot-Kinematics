from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():

    kinematic_model_node = Node(
        package='kinematic_model',
        executable='kinematic_model_node',
        output='screen',
        name='kinematic_model_node',
        parameters=[]
        )

    move_shelf_to_ship_node = Node(
        package='eight_trajectory',
        executable='eight_trajectory_node',
        output='screen',
        name='move_shelf_to_ship_node',
        )


    return LaunchDescription(
        [
            kinematic_model_node,
            move_shelf_to_ship_node
        ]
    )