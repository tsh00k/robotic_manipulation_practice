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
Run real oracle episodes while injecting rejected visual observations.

Build/source the workspace first, then use an otherwise unused ROS domain:
    ROS_DOMAIN_ID=84 /usr/bin/python3 \
        src/task_executor/test/oracle_attachment_probe.py --episodes 3

Owns and cleans up the real bridge/executor processes; no camera or GUI required.
Logs go to /tmp/oracle_attachment_probe_{mujoco_bridge,task_executor}.log.
"""

import argparse
import subprocess
import time
from pathlib import Path

import rclpy
from manipulation_interfaces.msg import BridgeObservation, EpisodeOutcome, VisionObjectPose
from rclpy.node import Node
from std_msgs.msg import Empty


class Probe(Node):
    def __init__(self):
        super().__init__('oracle_attachment_probe')
        self.outcome = None
        self.samples = []
        self.rejected = 0
        self.start = self.create_publisher(Empty, '/task_executor/start_episode', 10)
        self.vision = self.create_publisher(
            VisionObjectPose, '/object_pose_estimator/object_pose', 10)
        self.create_subscription(
            BridgeObservation, '/mujoco_bridge/episode_observation', self.on_bridge, 10)
        self.create_subscription(
            EpisodeOutcome, '/task_executor/episode_outcome', self.on_outcome, 10)

    def on_bridge(self, msg):
        self.samples.append(msg)
        vision = VisionObjectPose()
        vision.header = msg.object_pose.header
        vision.bridge_session = msg.bridge_session
        vision.generation = msg.generation
        vision.sample_sequence = msg.sample_sequence
        vision.evidence_state = VisionObjectPose.REJECTED
        vision.state_reason = 'CANDIDATE_INVALID'
        vision.diagnostic_stage = VisionObjectPose.DIAGNOSTIC_GEOMETRY
        self.vision.publish(vision)
        self.rejected += 1

    def on_outcome(self, msg):
        self.outcome = msg


def spin_until(probe, predicate, timeout):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.1)
    if not predicate():
        raise RuntimeError('probe timed out; inspect /tmp/oracle_attachment_probe_*.log')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episodes', type=int, default=3)
    args = parser.parse_args()
    if args.episodes < 1:
        parser.error('--episodes must be positive')
    root = Path(__file__).resolve().parents[3]
    rclpy.init()
    probe = Probe()
    processes = []
    logs = []
    try:
        for package, name, options in [
            ('mujoco_bridge', 'mujoco_bridge_node', []),
            ('task_executor', 'task_executor_node', [
                '--ros-args', '-p', 'use_sim_time:=true', '-p',
                'observation_source:=oracle', '-p', 'verify.place_region_radius_m:=0.03']),
        ]:
            log = open(f'/tmp/oracle_attachment_probe_{package}.log', 'w')
            logs.append(log)
            executable = root / 'install' / package / 'lib' / package / name
            processes.append(subprocess.Popen(
                [str(executable)] + options, stdout=log, stderr=subprocess.STDOUT))
        spin_until(probe, lambda: (
            bool(probe.samples) and probe.start.get_subscription_count() > 0 and
            probe.vision.get_subscription_count() > 0), 15)
        assert probe.count_publishers('/clock') == 1
        previous_generation = 0
        for episode in range(args.episodes):
            probe.outcome = None
            probe.samples.clear()
            rejected_before = probe.rejected
            probe.start.publish(Empty())
            spin_until(probe, lambda: probe.outcome is not None, 120)
            result = probe.outcome
            # Ignore pre-reset DDS samples queued when the start command was sent.
            generation = max(sample.generation for sample in probe.samples)
            samples = [s for s in probe.samples if s.generation == generation]
            attached = [s for s in samples
                        if s.attachment_state == BridgeObservation.ATTACHMENT_ATTACHED]
            rejected_count = probe.rejected - rejected_before
            print(
                f'episode={episode + 1} success={result.success} '
                f'failure={result.failure_code} retries={result.retries} '
                f'phases={list(result.phase_names)} rejected_injected={rejected_count} '
                f'first_attached_z='
                f'{attached[0].object_pose.pose.position.z if attached else None}',
                flush=True)
            assert result.success and result.retries == 0, result.failure_code
            assert result.observation_source == 'oracle'
            assert rejected_count > 0
            assert generation > previous_generation
            assert attached and attached[0].object_pose.pose.position.z < 0.26
            assert max(s.object_pose.pose.position.z for s in attached) > 0.26
            assert any(s.attachment_state == BridgeObservation.ATTACHMENT_RELEASED
                       for s in samples)
            assert any(s.attachment_state == BridgeObservation.ATTACHMENT_NOT_ATTACHED
                       for s in samples)
            previous_generation = generation
    finally:
        for proc in processes:
            proc.terminate()
        for proc in processes:
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        for log in logs:
            log.close()
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
