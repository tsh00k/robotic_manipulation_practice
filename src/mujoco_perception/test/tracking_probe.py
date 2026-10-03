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
"""
Drive an oracle baseline while independently recording vision evidence.

The depth relay injects bounded blackouts. Truth and task outcome are used only
in this offline probe, never fed back into the estimator.
"""

import argparse
from collections import Counter
import gzip
import json
from pathlib import Path
import re
import subprocess
import time

import numpy as np
import rclpy
from manipulation_interfaces.msg import BridgeObservation, EpisodeOutcome, VisionObjectPose
from manipulation_interfaces.srv import ResetScene
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.serialization import serialize_message
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty
from tf2_msgs.msg import TFMessage


class Probe(Node):
    def __init__(self, destination):
        super().__init__('tracking_probe')
        self.origin = time.monotonic()
        self.sent = {}
        self.rows = []
        self.outcomes = []
        self.episode_start_wall = float('inf')
        self.parameters = {}
        self.parameter_names = ['tracking.min_confidence', 'box_size_y_m',
                                'robot_mask.depth_tolerance_m']
        self.injection_index = 0
        self.blackout_start_s = None
        self.next_blackout_s = 0.0
        self.recording = destination / 'recording'
        self.recording.mkdir(exist_ok=True)
        self.inputs = {}
        self.attachment_states = {}
        self.pending_recordings = set()
        self.recorded_frames = 0
        for name, topic, kind in [
                ('rgb', '/mujoco_bridge/camera/color/image_raw', Image),
                ('color_info', '/mujoco_bridge/camera/color/camera_info', CameraInfo),
                ('depth_info', '/mujoco_bridge/camera/depth/camera_info', CameraInfo),
                ('observation', '/mujoco_bridge/episode_observation', BridgeObservation),
                ('tf', '/tf', TFMessage)]:
            self.inputs[name] = {}
            self.create_subscription(
                kind, topic, lambda msg, name=name: self.cache(name, msg),
                100 if name in ['observation', 'tf'] else qos_profile_sensor_data)
        self.create_subscription(
            TFMessage, '/tf_static', self.static_tf,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.inputs['depth'] = {}

    def cache(self, name, message):
        header = (message.joint_state.header if name == 'observation' else
                  message.transforms[0].header if name == 'tf' else message.header)
        key = (header.stamp.sec, header.stamp.nanosec)
        if name == 'observation':
            self.attachment_states[key] = message.attachment_state
        self.inputs[name][key] = serialize_message(message)
        self.save_pending()
        while len(self.inputs[name]) > (300 if name == 'observation' else 100):
            self.inputs[name].pop(next(iter(self.inputs[name])))

    def static_tf(self, message):
        with gzip.open(self.recording / 'static_tf.cdr.gz', 'wb') as output:
            output.write(serialize_message(message))

    def setup_subscriptions(self):
        self.reset_client = self.create_client(ResetScene, '/mujoco_bridge/reset_with_generation')
        self.parameter_client = self.create_client(
            GetParameters, '/object_pose_estimator/get_parameters')
        self.depth_pub = self.create_publisher(
            Image, '/mujoco_bridge/camera/depth/image_raw', qos_profile_sensor_data)
        self.start_pub = self.create_publisher(Empty, '/task_executor/start_episode', 10)
        self.create_subscription(
            Image, '/stage5/raw_depth', self.depth, qos_profile_sensor_data)
        self.create_subscription(
            VisionObjectPose, '/object_pose_estimator/object_pose', self.pose, 10)
        self.create_subscription(
            EpisodeOutcome, '/task_executor/episode_outcome', self.outcome, 10)

    def depth(self, message):
        key = (message.header.stamp.sec, message.header.stamp.nanosec)
        sim_time_s = key[0] + key[1] * 1e-9
        if (self.injection_index < 2 and self.blackout_start_s is None
                and len(self.rows) >= 3 and self.rows[-1]['state'] == 'MEASURED'
                and sim_time_s >= self.next_blackout_s):
            self.blackout_start_s = sim_time_s
        blackout = False
        if self.blackout_start_s is not None:
            duration_s = [0.2, 0.8][self.injection_index]
            blackout = sim_time_s - self.blackout_start_s < duration_s
            if not blackout:
                self.injection_index += 1
                self.blackout_start_s = None
                self.next_blackout_s = sim_time_s + 0.5
        if blackout:
            values = np.full((message.height, message.step // 4), np.nan, dtype=np.float32)
            message.data = values.tobytes()
        self.sent[key] = (time.monotonic(), blackout)
        self.depth_pub.publish(message)
        self.cache('depth', message)
        while len(self.sent) > 100:
            self.sent.pop(next(iter(self.sent)))

    def pose(self, message):
        key = (message.header.stamp.sec, message.header.stamp.nanosec)
        arrival, blackout = self.sent.get(key, (time.monotonic(), False))
        latency_ms = (time.monotonic() - arrival) * 1000
        arrival_wall_s = time.time()
        self.pending_recordings.add(key)
        self.save_pending()
        self.rows.append({
            'arrival_wall_s': arrival_wall_s,
            'time_s': key[0] + key[1] * 1e-9,
            'session': message.bridge_session,
            'generation': message.generation,
            'sequence': message.sample_sequence,
            'state': ['REJECTED', 'MEASURED', 'PREDICTED', 'OCCLUDED'][message.evidence_state],
            'attachment_state': self.attachment_states.get(key, 0),
            'grasp_state': message.grasp_state,
            'attachment_valid': message.attachment_valid,
            'measurement_sequence': message.last_measurement_sequence,
            'points': message.point_count,
            'confidence': message.confidence,
            'residual_m': message.residual_m,
            'processing_ms': message.processing_ms,
            'relay_to_result_ms': latency_ms,
            'state_reason': message.state_reason,
            'diagnostic_stage': message.diagnostic_stage,
            'candidate_count': message.candidate_count,
            'eligible_candidate_count': message.eligible_candidate_count,
            'support_prior_used': message.support_prior_used,
            'xyz': [message.pose.position.x, message.pose.position.y, message.pose.position.z],
            'blackout': blackout,
        })

    def save_pending(self):
        for key in sorted(self.pending_recordings):
            if not all(key in cache for cache in self.inputs.values()):
                continue
            frame_dir = self.recording / f'{key[0]}_{key[1]:09d}'
            frame_dir.mkdir(exist_ok=True)
            for name, cache in self.inputs.items():
                with gzip.open(frame_dir / (name + '.cdr.gz'), 'wb') as output:
                    output.write(cache[key])
            robot_keys = sorted(k for k in self.inputs['observation'] if k <= key)
            with gzip.open(frame_dir / 'robot_observations.json.gz', 'wt',
                           encoding='ascii') as output:
                json.dump([self.inputs['observation'][k].hex() for k in robot_keys], output)
            self.pending_recordings.remove(key)
            self.recorded_frames += 1

    def outcome(self, message):
        self.outcomes.append({'success': message.success, 'source': message.observation_source})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=65.0)
    parser.add_argument('--out', default='log/stage5_probe')
    args = parser.parse_args()
    destination = Path(args.out)
    destination.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    probe = Probe(destination)
    probe.setup_subscriptions()
    processes = []
    files = []
    try:
        commands = {
            'bridge': ['install/mujoco_bridge/lib/mujoco_bridge/mujoco_bridge_node',
                       '--ros-args', '-p', 'enable_rgbd_camera:=true', '-r',
                       '/mujoco_bridge/camera/depth/image_raw:=/stage5/raw_depth'],
            'perception': ['install/mujoco_perception/lib/mujoco_perception/'
                           'object_pose_estimator_node', '--ros-args', '-p', 'use_sim_time:=true'],
            'executor': ['install/task_executor/lib/task_executor/task_executor_node',
                         '--ros-args', '-p', 'use_sim_time:=true'],
        }
        for name, command in commands.items():
            output = (destination / (name + '.log')).open('w', encoding='utf-8')
            files.append(output)
            processes.append(subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT))
        started = False
        reset_future = None
        parameter_future = None
        bootstrapped = False
        while time.monotonic() - probe.origin < args.seconds:
            rclpy.spin_once(probe, timeout_sec=0.05)
            if parameter_future is None and probe.parameter_client.service_is_ready():
                request = GetParameters.Request()
                request.names = probe.parameter_names
                parameter_future = probe.parameter_client.call_async(request)
            if parameter_future is not None and parameter_future.done() and not probe.parameters:
                probe.parameters = {name: value.double_value for name, value in zip(
                    probe.parameter_names, parameter_future.result().values)}
            if reset_future is None and probe.reset_client.service_is_ready():
                reset_future = probe.reset_client.call_async(ResetScene.Request())
            if reset_future is not None and reset_future.done() and not bootstrapped:
                assert reset_future.result().success, 'Initial scene reset failed'
                bootstrapped = True
            if (bootstrapped and not started and time.monotonic() - probe.origin > 8.0
                    and probe.injection_index == 2
                    and probe.start_pub.get_subscription_count() > 0):
                probe.start_pub.publish(Empty())
                probe.episode_start_wall = time.time()
                started = True
            if any(process.poll() is not None for process in processes):
                raise RuntimeError('A validation node exited unexpectedly')
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for output in files:
            output.close()

    transitions = []
    pattern = re.compile(r'\[([0-9]+\.[0-9]+)\].*phase (\w+) -> (\w+)')
    for line in (destination / 'executor.log').read_text(encoding='utf-8').splitlines():
        match = pattern.search(line)
        if match:
            transitions.append((float(match.group(1)), match.group(3)))
    for row in probe.rows:
        row['phase'] = 'HOME' if row['arrival_wall_s'] >= probe.episode_start_wall else 'IDLE'
        for stamp, phase in transitions:
            if stamp <= row['arrival_wall_s']:
                row['phase'] = phase
    counts = {}
    for phase in sorted({row['phase'] for row in probe.rows}):
        counts[phase] = dict(Counter(row['state'] for row in probe.rows if row['phase'] == phase))
    summary = {'frames': len(probe.rows), 'counts_by_phase': counts,
               'recorded_frames': probe.recorded_frames,
               'unrecorded_missing_inputs': {
                   name: sum(key not in cache for key in probe.pending_recordings)
                   for name, cache in probe.inputs.items()},
               'outcomes': probe.outcomes,
               'parameters': probe.parameters,
               'attached_frames': sum(row['attachment_valid'] for row in probe.rows)}
    for field in ['processing_ms', 'relay_to_result_ms']:
        percentiles = np.percentile([r[field] for r in probe.rows], [50, 95]).tolist()
        summary[field] = dict(zip(['p50', 'p95'], percentiles))
    (destination / 'frames.json').write_text(json.dumps(probe.rows, indent=2), encoding='utf-8')
    (destination / 'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print(json.dumps(summary, indent=2))
    assert probe.rows, 'No paired vision frames'
    assert any(row['state'] == 'OCCLUDED' and row['blackout'] for row in probe.rows)
    for row in probe.rows:
        if row['state'] == 'MEASURED':
            assert row['points'] >= 3 and row['state'] == 'MEASURED'
            assert row['sequence'] == row['measurement_sequence']
        else:
            assert row['state'] != 'MEASURED' and np.isnan(row['residual_m'])
        if row['state'] in ['OCCLUDED', 'REJECTED']:
            assert row['state'] in ('REJECTED', 'OCCLUDED')
    assert not any(row['state'] == 'PREDICTED' for row in probe.rows)
    probe.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
