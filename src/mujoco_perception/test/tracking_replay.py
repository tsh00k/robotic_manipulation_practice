#!/usr/bin/python3
# Copyright 2026 anby
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Replay captured perception inputs without a simulator or task executor."""

import argparse
from collections import Counter
import gzip
import json
from pathlib import Path
import subprocess
import time

import rclpy
from manipulation_interfaces.msg import BridgeObservation, VisionObjectPose
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--prediction-age', type=float, default=0.3)
    parser.add_argument('--held-age', type=float, default=5.0)
    parser.add_argument('--invalid-tcp', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = Node('tracking_replay')
    rows = []
    node.create_subscription(
        VisionObjectPose, '/object_pose_estimator/object_pose', rows.append, 100)
    channels = [
        ('observation', '/mujoco_bridge/episode_observation', BridgeObservation, 10),
        ('tf', '/tf', TFMessage, 100),
        ('color_info', '/mujoco_bridge/camera/color/camera_info', CameraInfo,
         qos_profile_sensor_data),
        ('depth_info', '/mujoco_bridge/camera/depth/camera_info', CameraInfo,
         qos_profile_sensor_data),
        ('rgb', '/mujoco_bridge/camera/color/image_raw', Image, qos_profile_sensor_data),
        ('depth', '/mujoco_bridge/camera/depth/image_raw', Image, qos_profile_sensor_data),
    ]
    publishers = {name: node.create_publisher(kind, topic, qos)
                  for name, topic, kind, qos in channels}
    static_pub = node.create_publisher(
        TFMessage, '/tf_static',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    command = ['install/mujoco_perception/lib/mujoco_perception/object_pose_estimator_node',
               '--ros-args', '-p', f'tracking.max_prediction_age_s:={args.prediction_age}',
               '-p', f'tracking.held_prediction_age_s:={args.held_age}']
    with (args.recording.parent / 'replay.log').open('w', encoding='utf-8') as output:
        process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 10
            while any(pub.get_subscription_count() == 0 for pub in publishers.values()):
                rclpy.spin_once(node, timeout_sec=0.1)
                if time.monotonic() > deadline or process.poll() is not None:
                    raise RuntimeError('Estimator did not connect')
            with gzip.open(args.recording / 'static_tf.cdr.gz', 'rb') as source:
                static_pub.publish(deserialize_message(source.read(), TFMessage))
            rclpy.spin_once(node, timeout_sec=0.2)
            frames = sorted((p for p in args.recording.iterdir() if p.is_dir()),
                            key=lambda p: tuple(map(int, p.name.split('_'))))
            last_robot_time = -1.0
            for frame in frames:
                before = len(rows)
                robot_path = frame / 'robot_observations.json.gz'
                if robot_path.exists():
                    with gzip.open(robot_path, 'rt', encoding='ascii') as source:
                        for encoded in json.load(source):
                            robot = deserialize_message(bytes.fromhex(encoded), BridgeObservation)
                            if args.invalid_tcp:
                                robot.world_to_hand_tcp.transform.rotation.w = 0.0
                                robot.world_to_hand_tcp.transform.rotation.x = 0.0
                                robot.world_to_hand_tcp.transform.rotation.y = 0.0
                                robot.world_to_hand_tcp.transform.rotation.z = 0.0
                            stamp = robot.joint_state.header.stamp
                            robot_time = stamp.sec + stamp.nanosec * 1e-9
                            if robot_time <= last_robot_time:
                                continue
                            publishers['observation'].publish(robot)
                            last_robot_time = robot_time
                            rclpy.spin_once(node, timeout_sec=0.005)
                for name, _, kind, _ in channels:
                    with gzip.open(frame / (name + '.cdr.gz'), 'rb') as source:
                        message = deserialize_message(source.read(), kind)
                        if name == 'observation' and args.invalid_tcp:
                            message.world_to_hand_tcp.transform.rotation.w = 0.0
                            message.world_to_hand_tcp.transform.rotation.x = 0.0
                            message.world_to_hand_tcp.transform.rotation.y = 0.0
                            message.world_to_hand_tcp.transform.rotation.z = 0.0
                        publishers[name].publish(message)
                deadline = time.monotonic() + 1.0
                while len(rows) == before and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.02)
                assert len(rows) > before, f'No replay result for {frame.name}'
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
    counts = Counter(['REJECTED', 'MEASURED', 'PREDICTED', 'OCCLUDED'][r.evidence_state]
                     for r in rows)
    trace = [{'time_s': r.header.stamp.sec + r.header.stamp.nanosec * 1e-9,
              'sequence': r.sample_sequence, 'state': r.evidence_state,
              'grasp_state': r.grasp_state, 'attachment_valid': r.attachment_valid,
              'grasp_reason': r.grasp_reason, 'reason': r.rejection_reason,
              'prediction_age_s': r.prediction_age_s} for r in rows]
    (args.recording.parent / 'replay_trace.json').write_text(
        json.dumps(trace, indent=2), encoding='utf-8')
    print(json.dumps({'frames': len(rows), 'states': dict(counts),
                      'attached_frames': sum(r.attachment_valid for r in rows),
                      'grasp_reasons': dict(Counter(r.grasp_reason for r in rows)),
                      'prediction_age_limit_s': args.prediction_age}, indent=2))
    assert rows, 'Empty replay'
    for row in rows:
        assert row.accepted == (row.evidence_state == row.MEASURED)
        if row.evidence_state == row.PREDICTED:
            if row.attachment_valid:
                assert row.prediction_age_s <= args.held_age + 1e-9
            elif row.grasp_state != row.GRASP_RELEASED:
                assert row.prediction_age_s <= args.prediction_age + 1e-9
        if row.evidence_state in [row.REJECTED, row.OCCLUDED]:
            assert not row.pose_valid
    if args.invalid_tcp:
        assert all(row.evidence_state == row.REJECTED for row in rows)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
