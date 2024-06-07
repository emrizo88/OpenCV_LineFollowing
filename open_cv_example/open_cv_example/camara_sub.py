from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='proyecto_final',
            executable='semaforo',
            name='semaforo_node',
            output='screen',
            parameters=[]  # Add parameters if needed
        ),
        Node(
            package='proyecto_final',
            executable='detector_objetos',
            name='detector_objetos_node',
            output='screen',
            parameters=[]  # Add parameters if needed
        ),
        Node(
            package='proyecto_final',
            executable='velocidad',
            name='velocidad_node',
            output='screen',
            parameters=[]  # Add parameters if needed
        ),
        Node(
            package='proyecto_final',
            executable='nodes',
            name='nodes_node',
            output='screen',
            parameters=[]  # Add parameters if needed
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
