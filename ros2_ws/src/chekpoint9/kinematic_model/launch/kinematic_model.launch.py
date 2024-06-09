from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():

    wheel_velocities = Node(
        package='wheel_velocities_publisher',
        executable='wheel_velocities_publisher',
        output='screen',
        name='wheel_velocities',
        parameters=[]
        )

    kinematic_model_node = Node(
        package='kinematic_model',
        executable='kinematic_model_node',
        output='screen',
        name='kinematic_model_node',
        parameters=[]
        )


    return LaunchDescription(
        [
            wheel_velocities,
            kinematic_model_node
        ]
    )