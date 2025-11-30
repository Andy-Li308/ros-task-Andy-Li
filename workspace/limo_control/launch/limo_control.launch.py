from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
   
    # Path to limo_simulation's launch file
    limo_simulation_launch = os.path.join(
        get_package_share_directory('limo_simulation'),
        'launch',
        'limo.launch.py'
    )

    # Include that launch file
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(limo_simulation_launch)
    )

    #launch the controller node
    controller_node = Node(
            package='limo_control',
            executable='limo_control_node',
            name='P_Controller_Node',
            output='screen',
        )
    
    return LaunchDescription([
        simulation_launch,
        controller_node
    ])