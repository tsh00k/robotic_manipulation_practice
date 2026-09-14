from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution(
            [FindPackageShare('mujoco_bridge'), 'rviz', 'demo.rviz']
        ),
    )

    bridge_node = Node(
        package='mujoco_bridge',
        executable='mujoco_bridge_node',
        name='mujoco_bridge',
        output='screen',
        # This node *is* the /clock source, so use_sim_time is a no-op for it (it
        # never calls get_clock()->now() -- see mujoco_bridge_node.cpp simTime()).
        # Set anyway to establish the pattern: every node added to this launch file
        # going forward (task_executor in Stage I) must set it too, or its timeouts
        # run on wall time instead of sim time.
        parameters=[{'use_sim_time': True}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen',
        # Hardware-accelerated GL through this dev container's forwarded X11 socket
        # (VSCode Remote-Containers -> Xwayland on the host) hangs indefinitely
        # during context creation -- confirmed by watching rviz2 sit idle on all
        # threads for 24s+ with zero log output and no window ever mapped. Forcing
        # Mesa's software rasterizer (llvmpipe) skips that negotiation entirely and
        # the window appears in ~2s. See week1.md Stage E for the full diagnosis.
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
    )

    return LaunchDescription([rviz_config_arg, bridge_node, rviz_node])
