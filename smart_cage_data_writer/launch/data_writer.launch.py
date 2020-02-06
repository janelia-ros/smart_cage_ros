import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'node_prefix',
            default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
            description='Prefix for node names'),
        launch_ros.actions.Node(
            package='smart_cage_data_writer', node_executable='lickport_data_writer_node', output='screen',
            node_name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'lickport_data_writer_node']),
        launch_ros.actions.Node(
            package='smart_cage_data_writer', node_executable='tunnel_data_writer_node', output='screen',
            node_name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'tunnel_data_writer_node']),
    ])
