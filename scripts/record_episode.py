#!/usr/bin/python3
"""Record one pick-and-place episode as a raw record for the VLA dataset (Week 5 Stage 6).

Assumes the demo is already running (one mujoco_bridge_node, one task_executor_node, and the
estimator for the vision source), e.g. `ros2 launch mujoco_bridge demo.launch.py
scene_enabled:=true`. Starts `ros2 bag record` on an explicit topic list, starts one episode,
waits for its outcome, stops the recorder, and writes next to the bag:

  metadata.json  the episode metadata of the dataset contract (task, objects, targets; the
                 contract's schema allows nothing else)
  sidecar.json   everything else a converter or a reviewer needs and the contract has no field
                 for: the layout the bridge was given, the outcome, the latched box and the grasp
                 tool yaw with its distance to the +-45 degree switch (C8), the code version.

The frames themselves -- RGB, depth, state and action at 10 Hz -- are in the bag: the camera
topics and BridgeObservation (joint_state = state, arm_command + gripper_command_width_m =
action, all stamped with the same physics step). The converter pairs them by exact stamp.

Must be run with /usr/bin/python3 (CLAUDE.md, "变体二").

Usage:
    /usr/bin/python3 scripts/record_episode.py [--out results/episodes] [--label NAME]
"""
import argparse
import datetime
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from manipulation_interfaces.msg import EpisodeOutcome, InitialPoseLatch
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import Empty

TOPICS = [
    '/mujoco_bridge/camera/color/image_raw',
    '/mujoco_bridge/camera/depth/image_raw',
    '/mujoco_bridge/camera/color/camera_info',
    '/mujoco_bridge/camera/depth/camera_info',
    '/mujoco_bridge/episode_observation',
    '/mujoco_bridge/joint_command',
    '/mujoco_bridge/gripper_command',
    '/task_executor/initial_pose_latch',
    '/task_executor/episode_outcome',
    '/tf_static',
]
# The camera topics are best effort (SensorDataQoS); record them the same way, so that the
# recorder's subscription matches the publisher's QoS whatever rosbag2 would infer.
BEST_EFFORT = [topic for topic in TOPICS if topic.startswith('/mujoco_bridge/camera/')]
TASK = 'put the red box into the bin'
OBJECTS = [{'id': 'box', 'class': 'box', 'color': 'red'}]
TARGETS = [{'id': 'bin', 'class': 'bin', 'color': 'teal'}]
SCENE_PARAMETERS = [f'scene.{o}.{f}' for o in ('box', 'bin')
                    for f in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')]


def count_processes(prefix):
    out = subprocess.run(['ps', '-eo', 'comm'], capture_output=True, text=True).stdout
    return sum(1 for line in out.split() if line.startswith(prefix))


def spin_until(node, predicate, timeout):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    return predicate()


def get_parameters(node, target, names):
    client = node.create_client(GetParameters, f'/{target}/get_parameters')
    if not client.wait_for_service(timeout_sec=5.0):
        return {}
    future = client.call_async(GetParameters.Request(names=names))
    spin_until(node, future.done, 5.0)
    values = {}
    for name, value in zip(names, future.result().values if future.result() else []):
        # double_value for the scene poses; type 0 means the parameter is not set
        if value.type != 0:
            values[name] = value.double_value
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--out', default='results/episodes')
    parser.add_argument('--label', default='')
    parser.add_argument('--timeout-s', type=float, default=300.0)
    args = parser.parse_args()

    # Exactly one instance of each node (STUDY_NOTES_GUIDE.md 3.1); `comm` is truncated.
    for prefix in ('mujoco_bridge_n', 'task_executor_n'):
        if count_processes(prefix) != 1:
            sys.exit(f'refusing to record: expected exactly one {prefix}*, found '
                     f'{count_processes(prefix)}; start the demo first')

    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    episode_dir = Path(args.out) / (stamp + (f'_{args.label}' if args.label else ''))
    episode_dir.mkdir(parents=True)
    overrides = episode_dir / 'qos_overrides.yaml'
    overrides.write_text(''.join(
        f'{topic}:\n  reliability: best_effort\n  history: keep_last\n  depth: 50\n'
        for topic in BEST_EFFORT))

    rclpy.init()
    node = rclpy.create_node('record_episode')
    outcomes, latches = [], []
    node.create_subscription(EpisodeOutcome, '/task_executor/episode_outcome', outcomes.append, 10)
    node.create_subscription(InitialPoseLatch, '/task_executor/initial_pose_latch',
                             latches.append, 50)
    start = node.create_publisher(Empty, '/task_executor/start_episode', 10)
    if node.count_publishers('/clock') > 1:
        sys.exit('refusing to record: more than one /clock publisher')

    bag_log = open(episode_dir / 'ros2_bag_record.log', 'w')
    recorder = subprocess.Popen(
        ['ros2', 'bag', 'record', '-o', str(episode_dir / 'bag'), '--use-sim-time',
         '--qos-profile-overrides-path', str(overrides)] + TOPICS,
        stdout=bag_log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        # The recorder is ready once it subscribes to the observation stream.
        base = node.count_subscribers('/mujoco_bridge/episode_observation')
        if not spin_until(node, lambda: node.count_subscribers(
                '/mujoco_bridge/episode_observation') > base, 30):
            sys.exit('ros2 bag record did not come up; see ' + str(bag_log.name))
        spin_until(node, lambda: False, 1.0)
        if not spin_until(node, lambda: start.get_subscription_count() > 0, 30):
            sys.exit('task_executor did not come up')
        start.publish(Empty())
        if not spin_until(node, lambda: bool(outcomes), args.timeout_s):
            sys.exit(f'no outcome within {args.timeout_s} s')
        # The release is followed by a few more frames; keep them.
        spin_until(node, lambda: False, 1.0)
    finally:
        os.killpg(recorder.pid, signal.SIGINT)
        try:
            recorder.wait(30)
        except subprocess.TimeoutExpired:
            os.killpg(recorder.pid, signal.SIGKILL)
            recorder.wait()

    outcome = outcomes[-1]
    layout = get_parameters(node, 'mujoco_bridge', SCENE_PARAMETERS)
    latched = [m for m in latches if m.state == InitialPoseLatch.LATCHED]
    box_yaw = math.degrees(latched[-1].box.yaw_rad) if latched else None
    grasp_yaw = None if box_yaw is None else (box_yaw + 45.0) % 90.0 - 45.0
    node.destroy_node()
    rclpy.shutdown()

    (episode_dir / 'metadata.json').write_text(json.dumps(
        {'task': TASK, 'objects': OBJECTS, 'targets': TARGETS}, indent=2) + '\n')
    commit = subprocess.run(['git', 'rev-parse', 'HEAD'], capture_output=True,
                            text=True).stdout.strip()
    dirty = bool(subprocess.run(['git', 'status', '--porcelain', '--untracked-files=no'],
                                capture_output=True, text=True).stdout.strip())
    (episode_dir / 'sidecar.json').write_text(json.dumps({
        'recorded_utc': stamp,
        'label': args.label,
        'git_commit': commit,
        'git_dirty': dirty,
        'layout': layout,
        'outcome': {
            'success': outcome.success, 'failure_code': outcome.failure_code,
            'retries': outcome.retries, 'observation_source': outcome.observation_source,
            'observation_failure_reason': outcome.observation_failure_reason,
            'carry_width_alert': outcome.carry_width_alert,
            'phase_names': list(outcome.phase_names),
            'phase_durations_s': list(outcome.phase_durations_s)},
        'latched_box_yaw_deg': box_yaw,
        # C8: the tool turns with the box yaw folded to [-45, 45); near +-45 two almost equal
        # boxes are grasped 90 degrees apart, so a converter may filter on the distance.
        'grasp_tool_yaw_deg': grasp_yaw,
        'grasp_yaw_distance_to_switch_deg': None if grasp_yaw is None else 45.0 - abs(grasp_yaw),
        'topics': TOPICS,
    }, indent=2) + '\n')
    print(f'{episode_dir}: success={outcome.success} failure={outcome.failure_code} '
          f'retries={outcome.retries}')
    return 0 if outcome.success else 1


if __name__ == '__main__':
    sys.exit(main())
