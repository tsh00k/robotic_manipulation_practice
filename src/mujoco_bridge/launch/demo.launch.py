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

    # Defaults below must match mujoco_bridge_node.cpp's declare_parameter() defaults
    # exactly, so omitting a launch argument reproduces today's behavior unchanged.
    # Scope is deliberately narrow (these four, not every declare_parameter() in the
    # node): joint_state_rate_hz/tf_rate_hz is week1.md's reverse-checklist item
    # 13.2 ("launch 文件目前没有暴露节点自己的 ROS 参数"), and the debug viewer args
    # are the Stage J feature that just made that item's trigger condition live --
    # both need to be flippable from the command line to run different demo
    # configurations without editing source. task_executor's fsm./grasp./verify.
    # params are measured calibration constants (Stage H/I), not per-run knobs, and
    # stay un-exposed until something actually needs them overridden at launch time.
    joint_state_rate_hz_arg = DeclareLaunchArgument('joint_state_rate_hz', default_value='100.0')
    tf_rate_hz_arg = DeclareLaunchArgument('tf_rate_hz', default_value='100.0')
    enable_debug_viewer_arg = DeclareLaunchArgument('enable_debug_viewer', default_value='false')
    debug_viewer_rate_hz_arg = DeclareLaunchArgument('debug_viewer_rate_hz', default_value='30.0')

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
        parameters=[{
            'use_sim_time': True,
            'joint_state_rate_hz': LaunchConfiguration('joint_state_rate_hz'),
            'tf_rate_hz': LaunchConfiguration('tf_rate_hz'),
            'enable_debug_viewer': LaunchConfiguration('enable_debug_viewer'),
            'debug_viewer_rate_hz': LaunchConfiguration('debug_viewer_rate_hz'),
        }],
        # Same fix, same reason as rviz_node's additional_env below: with
        # enable_debug_viewer:=true, this process also creates a GLFW/GL window
        # (DebugViewer, week2.md Stage J 11.7), and hardware-accelerated context
        # creation hangs indefinitely in this container -- see 11.7.4. Unlike
        # rviz2 that hang is fatal to the whole node, not just the view: it
        # happens synchronously inside the constructor, before rclcpp::spin()
        # ever starts, so /joint_states and /tf never come up either. Harmless
        # to set unconditionally: with enable_debug_viewer:=false (default) this
        # process never calls into GLFW/GL at all.
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
    )

    task_executor_node = Node(
        package='task_executor',
        executable='task_executor_node',
        name='task_executor',
        output='screen',
        # See bridge_node's own comment above -- this is that anticipated second
        # node. Without this, fsm.cpp's phase_timeout_s (and every
        # elapsed_in_phase_s measurement it is compared against) would run on wall
        # time instead of sim time.
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

    return LaunchDescription([
        rviz_config_arg,
        joint_state_rate_hz_arg,
        tf_rate_hz_arg,
        enable_debug_viewer_arg,
        debug_viewer_rate_hz_arg,
        bridge_node,
        task_executor_node,
        rviz_node,
    ])
