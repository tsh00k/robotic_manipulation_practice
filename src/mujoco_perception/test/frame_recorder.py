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
Record aligned camera frames, estimator debug images and bridge truth for offline study.

Starts its own bridge (camera on), pose estimator and an oracle-source executor, runs one
episode and saves one .npz with a row per camera frame:
    source install/setup.bash
    ROS_DOMAIN_ID=84 /usr/bin/python3 src/mujoco_perception/test/frame_recorder.py \
        --out /tmp/frames_default.npz
The executor runs in oracle mode on purpose: the episode then does not depend on the
estimator under study, which only has to publish its debug images. Truth fields (box pose,
TCP) are for offline evaluation and never go back into any node.
Use --box-yaw/--scene-param for a static capture (--no-episode) of a configured scene.
"""

import argparse
import os
import signal
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
from manipulation_interfaces.msg import BridgeObservation, EpisodeOutcome, VisionObjectPose
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty

BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=100)
IMAGE_TOPICS = {
    'rgb': '/mujoco_bridge/camera/color/image_raw',
    'depth': '/mujoco_bridge/camera/depth/image_raw',
    'mask': '/object_pose_estimator/debug/robot_mask',
    'filtered': '/object_pose_estimator/debug/filtered_depth',
}


def key_of(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def decode(msg):
    channels = {'rgb8': 3, 'mono8': 1, '32FC1': 1}[msg.encoding]
    dtype = np.float32 if msg.encoding == '32FC1' else np.uint8
    row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    width_bytes = msg.width * channels * np.dtype(dtype).itemsize
    array = row[:, :width_bytes].copy().view(dtype)
    return array.reshape(msg.height, msg.width, channels).squeeze()


def spin_until(node, predicate, timeout):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    return predicate()


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--out', required=True)
    parser.add_argument('--workspace', default=str(Path(__file__).resolve().parents[3]))
    parser.add_argument('--scene-param', action='append', default=[], metavar='NAME=VALUE',
                        help='bridge parameter, e.g. scene.enabled=true')
    parser.add_argument('--no-episode', action='store_true',
                        help='just let the scene settle and record --static-s seconds')
    parser.add_argument('--static-s', type=float, default=3.0)
    parser.add_argument('--no-estimator', action='store_true',
                        help='do not start the pose estimator (raw camera frames only)')
    args = parser.parse_args()

    install = Path(args.workspace) / 'install'
    env = dict(os.environ, LIBGL_ALWAYS_SOFTWARE='1')
    bridge_args = ['--ros-args', '-p', 'enable_rgbd_camera:=true']
    for item in args.scene_param:
        bridge_args += ['-p', item.replace('=', ':=', 1)]
    nodes = [('mujoco_bridge', 'mujoco_bridge_node', bridge_args)]
    if not args.no_estimator:
        nodes.append(('mujoco_perception', 'object_pose_estimator_node',
                      ['--ros-args', '-p', 'use_sim_time:=true']))
    if not args.no_episode:
        nodes.append(('task_executor', 'task_executor_node', [
            '--ros-args', '-p', 'use_sim_time:=true', '-p', 'observation_source:=oracle']))

    rclpy.init()
    node = rclpy.create_node('frame_recorder')
    images = {name: {} for name in IMAGE_TOPICS}
    bridge_rows = {}
    vision_rows = {}
    outcome = []
    for name, topic in IMAGE_TOPICS.items():
        node.create_subscription(
            Image, topic, lambda m, name=name: images[name].__setitem__(key_of(m.header.stamp), m),
            BEST_EFFORT)

    def on_bridge(m):
        pose = m.object_pose.pose
        tcp = m.world_to_hand_tcp.transform
        positions = dict(zip(m.joint_state.name, m.joint_state.position))
        bridge_rows[key_of(m.joint_state.header.stamp)] = dict(
            sequence=m.sample_sequence, attachment=m.attachment_state,
            width=positions.get('finger_joint1', np.nan) + positions.get('finger_joint2', np.nan),
            box=[pose.position.x, pose.position.y, pose.position.z,
                 pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
            tcp=[tcp.translation.x, tcp.translation.y, tcp.translation.z,
                 tcp.rotation.x, tcp.rotation.y, tcp.rotation.z, tcp.rotation.w],
            left=m.left_finger_contact, right=m.right_finger_contact)

    def on_vision(m):
        vision_rows[m.sample_sequence] = dict(
            state=m.evidence_state, reason=m.state_reason, points=m.point_count,
            confidence=m.confidence, candidates=m.candidate_count,
            eligible=m.eligible_candidate_count,
            pose=[m.pose.position.x, m.pose.position.y, m.pose.position.z])

    node.create_subscription(
        BridgeObservation, '/mujoco_bridge/episode_observation', on_bridge, 200)
    node.create_subscription(
        VisionObjectPose, '/object_pose_estimator/object_pose', on_vision, 200)
    node.create_subscription(EpisodeOutcome, '/task_executor/episode_outcome', outcome.append, 10)
    start = node.create_publisher(Empty, '/task_executor/start_episode', 10)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    if node.count_publishers('/clock') != 0:
        print('FAIL: a /clock publisher already exists; stop leftover nodes first')
        return 1

    processes, logs = [], []
    try:
        for package, name, arguments in nodes:
            log = open(f'/tmp/frame_recorder_{package}.log', 'w')
            logs.append(log)
            processes.append(subprocess.Popen(
                [str(install / package / 'lib' / package / name)] + arguments, stdout=log,
                stderr=subprocess.STDOUT, env=env, start_new_session=True))
        ready_stream = 'depth' if args.no_estimator else 'filtered'
        if not spin_until(node, lambda: bool(bridge_rows) and bool(images[ready_stream]), 90):
            print('FAIL: nodes did not come up; see /tmp/frame_recorder_*.log')
            return 1
        if args.no_episode:
            first = max(bridge_rows)
            spin_until(node, lambda: max(bridge_rows) - first > args.static_s * 1e9, 60)
        else:
            time.sleep(5.0)
            start.publish(Empty())
            if not spin_until(node, lambda: bool(outcome), 180):
                print('FAIL: episode did not finish')
                return 1
            spin_until(node, lambda: False, 1.5)  # let the last frames arrive
            print(f'episode success={outcome[-1].success} retries={outcome[-1].retries}')
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

    print('received per stream: ' + ', '.join(f'{n}={len(v)}' for n, v in images.items()) +
          f', bridge={len(bridge_rows)}, vision={len(vision_rows)}')
    # Camera frames can be missing from one stream (best-effort transport drops, and the
    # estimator publishes only what it processed). Keep every depth frame that has a bridge
    # sample, fill the missing images with zeros and record which ones are real.
    keys = sorted(k for k in images['depth'] if k in bridge_rows)
    seqs = [bridge_rows[k]['sequence'] for k in keys]
    shapes = {n: next(iter(images[n].values())) for n in IMAGE_TOPICS if images[n]}
    data = {}
    for name in IMAGE_TOPICS:
        if name not in shapes:
            continue
        sample = decode(shapes[name])
        data[name] = np.stack([decode(images[name][k]) if k in images[name]
                               else np.zeros_like(sample) for k in keys])
        data['has_' + name] = np.array([k in images[name] for k in keys])
    data['stamp_ns'] = np.array(keys)
    data['sequence'] = np.array(seqs)
    data['attachment'] = np.array([bridge_rows[k]['attachment'] for k in keys])
    data['width'] = np.array([bridge_rows[k]['width'] for k in keys])
    data['box'] = np.array([bridge_rows[k]['box'] for k in keys])
    data['tcp'] = np.array([bridge_rows[k]['tcp'] for k in keys])
    data['vision_state'] = np.array([vision_rows.get(s, {}).get('state', -1) for s in seqs])
    data['vision_points'] = np.array([vision_rows.get(s, {}).get('points', -1) for s in seqs])
    data['vision_confidence'] = np.array(
        [vision_rows.get(s, {}).get('confidence', np.nan) for s in seqs])
    data['vision_reason'] = np.array([vision_rows.get(s, {}).get('reason', 'NO_MESSAGE')
                                      for s in seqs])
    # Full-rate bridge samples too: the gripper width trace is not limited to 10 Hz.
    all_keys = sorted(bridge_rows)
    data['trace_stamp_ns'] = np.array(all_keys)
    data['trace_width'] = np.array([bridge_rows[k]['width'] for k in all_keys])
    data['trace_attachment'] = np.array([bridge_rows[k]['attachment'] for k in all_keys])
    np.savez_compressed(args.out, **data)
    print(f'saved {len(keys)} aligned frames ({len(all_keys)} bridge samples) to {args.out}')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
