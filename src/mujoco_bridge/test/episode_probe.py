#!/usr/bin/python3
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

"""
Run complete pick-and-place episodes against the real nodes and report each outcome.

The regression every Week 4.1 stage must keep: with default parameters, N episodes
succeed with zero retries, in oracle mode and in vision mode. Source the workspace under
test and use an otherwise unused ROS domain:
    source install/setup.bash
    ROS_DOMAIN_ID=84 /usr/bin/python3 src/mujoco_bridge/test/episode_probe.py \
        --source vision --episodes 3

Starts and reaps its own bridge, task_executor (and, for vision, the pose estimator);
nothing is launched through `ros2 run`. Logs go to /tmp/episode_probe_<node>.log. Vision
mode needs software GL (LIBGL_ALWAYS_SOFTWARE is set for the processes that render).
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from manipulation_interfaces.msg import BridgeObservation, EpisodeOutcome
from std_msgs.msg import Empty


def spin_until(node, predicate, timeout):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    return predicate()


def ros_params(items):
    arguments = ['--ros-args', '-p', 'use_sim_time:=true']
    for item in items:
        arguments += ['-p', item.replace('=', ':=', 1)]
    return arguments


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--source', choices=['oracle', 'vision'], default='oracle')
    parser.add_argument('--episodes', type=int, default=3)
    parser.add_argument('--workspace', default=str(Path(__file__).resolve().parents[3]))
    parser.add_argument('--bridge-param', action='append', default=[], metavar='NAME=VALUE')
    parser.add_argument('--executor-param', action='append', default=[], metavar='NAME=VALUE')
    parser.add_argument('--timeout-s', type=float, default=240.0, help='per episode, wall time')
    args = parser.parse_args()
    if args.episodes < 1:
        parser.error('--episodes must be positive')

    install = Path(args.workspace) / 'install'
    vision = args.source == 'vision'
    env = dict(os.environ, LIBGL_ALWAYS_SOFTWARE='1')
    bridge_params = list(args.bridge_param)
    if vision:
        bridge_params.append('enable_rgbd_camera=true')
    nodes = [
        ('mujoco_bridge', 'mujoco_bridge_node', ['--ros-args'] + sum(
            [['-p', p.replace('=', ':=', 1)] for p in bridge_params], [])),
        ('task_executor', 'task_executor_node',
         ros_params([f'observation_source={args.source}'] + args.executor_param)),
    ]
    if vision:
        nodes.append(('mujoco_perception', 'object_pose_estimator_node', ros_params([])))

    rclpy.init()
    probe = rclpy.create_node('episode_probe')
    outcome = []
    samples = []
    start = probe.create_publisher(Empty, '/task_executor/start_episode', 10)
    probe.create_subscription(
        BridgeObservation, '/mujoco_bridge/episode_observation', samples.append, 10)
    probe.create_subscription(
        EpisodeOutcome, '/task_executor/episode_outcome', outcome.append, 10)
    for _ in range(20):
        rclpy.spin_once(probe, timeout_sec=0.05)
    if probe.count_publishers('/clock') != 0:
        print('FAIL: a /clock publisher already exists; stop leftover nodes first')
        return 1

    processes, logs, results = [], [], []
    try:
        for package, name, arguments in nodes:
            log = open(f'/tmp/episode_probe_{package}.log', 'w')
            logs.append(log)
            executable = install / package / 'lib' / package / name
            processes.append(subprocess.Popen(
                [str(executable)] + arguments, stdout=log, stderr=subprocess.STDOUT,
                env=env, start_new_session=True))
        ready = spin_until(
            probe, lambda: bool(samples) and start.get_subscription_count() > 0, 60)
        if not ready or probe.count_publishers('/clock') != 1:
            print('FAIL: nodes did not come up; see /tmp/episode_probe_*.log')
            return 1
        if vision:
            # The estimator and the executor's vision subscription need a moment to match.
            time.sleep(5.0)
        for index in range(args.episodes):
            outcome.clear()
            began = time.monotonic()
            start.publish(Empty())
            if not spin_until(probe, lambda: bool(outcome), args.timeout_s):
                print(f'episode {index + 1}: TIMEOUT after {args.timeout_s:.0f} s')
                results.append(None)
                break
            result = outcome[-1]
            results.append(result)
            # Let the box settle after the last phase, then read where it ended up. This is
            # the simulator's ground truth, for the report only: nothing decides on it.
            settle_until = samples[-1].joint_state.header.stamp.sec + 2 if samples else 0
            spin_until(
                probe, lambda: samples and samples[-1].joint_state.header.stamp.sec >=
                settle_until, 30)
            final = samples[-1].object_pose.pose.position if samples else None
            where = (f'final_box(truth)=[{final.x:.3f} {final.y:.3f} {final.z:.3f}]'
                     if final else 'final_box(truth)=n/a')
            print(
                f'episode {index + 1}: success={result.success} '
                f'failure={result.failure_code or "-"} retries={result.retries} '
                f'source={result.observation_source} wall={time.monotonic() - began:.1f}s '
                f'{where}', flush=True)
            print(f'    phases={list(result.phase_names)}')
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
        for process in processes:
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for log in logs:
            log.close()
        probe.destroy_node()
        rclpy.shutdown()

    done = [r for r in results if r is not None]
    good = [r for r in done if r.success and r.retries == 0 and
            r.observation_source == args.source]
    print(f'SUMMARY source={args.source}: {len(good)}/{args.episodes} succeeded with zero '
          f'retries ({len(done)} finished)')
    return 0 if len(good) == args.episodes else 1


if __name__ == '__main__':
    sys.exit(main())
