import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='smart_cage', node_executable='smart_cage_node', output='screen',
            node_name='smart_cage_node'),
        launch_ros.actions.Node(
            package='lickport', node_executable='lickport_node', output='screen',
            node_name='lickport_node'),
        launch_ros.actions.Node(
            package='smart_cage_data_writer', node_executable='lickport_data_writer_node', output='screen',
            node_name='lickport_data_writer_node'),
        launch_ros.actions.Node(
            package='tunnel', node_executable='tunnel_node', output='screen',
            node_name='tunnel_node'),
        launch_ros.actions.Node(
            package='smart_cage_data_writer', node_executable='tunnel_data_writer_node', output='screen',
            node_name='tunnel_data_writer_node'),
    ])
