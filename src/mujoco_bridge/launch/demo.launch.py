# Copyright 2026 anby
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


SCENE_POSE_FIELDS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')


def scene_parameters(context):
    """
    Build the scene.* parameters for the bridge, and only the bridge.

    The box/bin start poses are simulator ground truth. Perception and the executor
    must get them from the camera (docs/adr/015), so these parameters are deliberately
    not passed to any other node. Only values the user gave are passed: 'auto' leaves
    the node's own default in place (x/y/angles: the layout of the legacy scene, z: the
    height that rests the lowest corner 1 mm above the table), so those numbers are
    written once, in the node.
    """
    enabled_text = LaunchConfiguration('scene_enabled').perform(context).strip().lower()
    if enabled_text not in ('true', 'false'):
        raise RuntimeError(
            f"launch argument scene_enabled:={enabled_text!r} must be true or false")
    enabled = enabled_text == 'true'
    parameters = {'scene.enabled': enabled}
    for body in ('box', 'bin'):
        for field in SCENE_POSE_FIELDS:
            name = f'{body}_{field}'
            text = LaunchConfiguration(name).perform(context).strip()
            if text == 'auto':
                continue
            if not enabled:
                raise RuntimeError(
                    f'launch argument {name}:={text} has no effect while scene_enabled is '
                    f'false; pass scene_enabled:=true')
            try:
                parameters[f'scene.{body}.{field}'] = float(text)
            except ValueError:
                raise RuntimeError(
                    f"launch argument {name}:={text!r} is not a number (metres or radians, "
                    f"or 'auto')") from None
    return parameters


def make_bridge_node(context):
    return [Node(
        package='mujoco_bridge',
        executable='mujoco_bridge_node',
        name='mujoco_bridge',
        output='screen',
        # This node *is* the /clock source, so use_sim_time is a no-op for it (it
        # never calls get_clock()->now() -- see mujoco_bridge_node.cpp simTime()).
        # Set anyway to establish the pattern: every node added to this launch file
        # going forward (task_executor in Stage I) must set it too, or its timeouts
        # run on wall time instead of sim time.
        parameters=[scene_parameters(context), {
            'use_sim_time': True,
            'joint_state_rate_hz': LaunchConfiguration('joint_state_rate_hz'),
            'tf_rate_hz': LaunchConfiguration('tf_rate_hz'),
            'enable_debug_viewer': LaunchConfiguration('enable_debug_viewer'),
            'debug_viewer_rate_hz': LaunchConfiguration('debug_viewer_rate_hz'),
            'enable_rgbd_camera': LaunchConfiguration('enable_rgbd_camera'),
            'camera_rate_hz': LaunchConfiguration('camera_rate_hz'),
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
    )]


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
    enable_rgbd_camera_arg = DeclareLaunchArgument('enable_rgbd_camera', default_value='false')
    camera_rate_hz_arg = DeclareLaunchArgument('camera_rate_hz', default_value='10.0')
    # vision is the demo's default from Week 4.1 Stage 11 on, when the vision episode first
    # completes into the bin; oracle remains for comparison.
    observation_source_arg = DeclareLaunchArgument('observation_source', default_value='vision')
    vision_min_confidence_arg = DeclareLaunchArgument('vision_min_confidence', default_value='0.5')
    vision_max_residual_arg = DeclareLaunchArgument('vision_max_residual_m', default_value='0.005')
    vision_min_inlier_arg = DeclareLaunchArgument('vision_min_inlier_ratio', default_value='0.7')

    # Optional box/bin start layout (bridge only, see scene_parameters). Off by default;
    # with it off the simulator is the legacy single-box scene. This moves objects in the
    # simulator only. The executor then puts the box into the bin (place.into_bin), taking the
    # bin from vision or, with the oracle source, from ~/ground_truth/bin_pose (Week 4.1
    # Stages 5-8). With vision the episode still ends at VERIFY until Stage 11.
    # Lengths in metres, angles in radians, R = Rz(yaw) Ry(pitch) Rx(roll); 'auto' = the
    # node's default. box_* is the box centre, bin_* the centre of the bin's inner floor.
    scene_args = [DeclareLaunchArgument('scene_enabled', default_value='false')]
    scene_args += [
        DeclareLaunchArgument(f'{body}_{field}', default_value='auto')
        for body in ('box', 'bin') for field in SCENE_POSE_FIELDS
    ]

    task_executor_node = Node(
        package='task_executor',
        executable='task_executor_node',
        name='task_executor',
        output='screen',
        # See bridge_node's own comment above -- this is that anticipated second
        # node. Without this, fsm.cpp's phase_timeout_s (and every
        # elapsed_in_phase_s measurement it is compared against) would run on wall
        # time instead of sim time.
        parameters=[{
            'use_sim_time': True,
            'observation_source': LaunchConfiguration('observation_source'),
            'vision.min_confidence': LaunchConfiguration('vision_min_confidence'),
            'vision.max_residual_m': LaunchConfiguration('vision_max_residual_m'),
            'vision.min_inlier_ratio': LaunchConfiguration('vision_min_inlier_ratio'),
            # A scene with the bin: put the box into the bin, whose pose comes from vision
            # (latched) or, with the oracle source, from the bridge's ground-truth bin pose
            # (Week 4.1 Stages 7, 8). The default scene has no bin and keeps the fixed target.
            'place.into_bin': LaunchConfiguration('scene_enabled'),
        }],
    )

    perception_node = Node(
        package='mujoco_perception',
        executable='object_pose_estimator_node',
        name='object_pose_estimator',
        output='screen',
        condition=LaunchConfigurationEquals('observation_source', 'vision'),
        parameters=[{'use_sim_time': True}],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
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
        enable_rgbd_camera_arg,
        camera_rate_hz_arg,
        observation_source_arg,
        vision_min_confidence_arg,
        vision_max_residual_arg,
        vision_min_inlier_arg,
        *scene_args,
        OpaqueFunction(function=make_bridge_node),
        task_executor_node,
        perception_node,
        rviz_node,
    ])
