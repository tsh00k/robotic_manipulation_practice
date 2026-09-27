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

"""Integration probe: stale observations must not drive a new episode.

Run after building and sourcing the workspace:
    /usr/bin/python3 src/task_executor/test/reset_generation_probe.py
"""

import subprocess
import time
from pathlib import Path

import rclpy
from manipulation_interfaces.msg import BridgeObservation, EpisodeOutcome
from manipulation_interfaces.srv import ResetScene
from rclpy.node import Node
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory


class Probe(Node):
    def __init__(self):
        super().__init__('reset_generation_probe')
        self.outcome = None
        self.command_count = 0
        self.reset_seen = False
        self.create_service(ResetScene, '/mujoco_bridge/reset_with_generation', self.on_reset)
        self.observation_pub = self.create_publisher(
            BridgeObservation, '/mujoco_bridge/episode_observation', 10)
        self.start_pub = self.create_publisher(Empty, '/task_executor/start_episode', 10)
        self.create_subscription(
            EpisodeOutcome, '/task_executor/episode_outcome', self.on_outcome, 10)
        self.create_subscription(
            JointTrajectory, '/mujoco_bridge/joint_command', self.on_command, 10)
        self.create_timer(0.05, self.publish_old_observation)

    def on_reset(self, request, response):
        self.reset_seen = True
        response.success = True
        response.message = 'probe reset'
        response.bridge_session = 7
        response.generation = 2
        return response

    def on_outcome(self, message):
        self.outcome = message

    def on_command(self, message):
        self.command_count += 1

    def publish_old_observation(self):
        if not self.reset_seen:
            return
        message = BridgeObservation()
        message.bridge_session = 7
        message.generation = 1
        message.sample_sequence = 100
        message.joint_state.name = [f'joint{i}' for i in range(1, 8)] + [
            'finger_joint1', 'finger_joint2']
        message.joint_state.position = [0.0] * 9
        message.joint_state.velocity = [0.0] * 9
        message.object_pose.header.frame_id = 'world'
        message.object_pose.pose.orientation.w = 1.0
        message.world_to_hand_tcp.header.frame_id = 'world'
        message.world_to_hand_tcp.child_frame_id = 'hand_tcp'
        message.world_to_hand_tcp.transform.rotation.w = 1.0
        self.observation_pub.publish(message)


def main():
    root = Path(__file__).resolve().parents[3]
    executable = root / 'install/task_executor/lib/task_executor/task_executor_node'
    rclpy.init()
    probe = Probe()
    executor = subprocess.Popen([str(executable)])
    try:
        deadline = time.monotonic() + 15.0
        while probe.start_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.1)
        if probe.start_pub.get_subscription_count() == 0:
            raise RuntimeError('executor start_episode subscriber did not appear')
        probe.start_pub.publish(Empty())
        while probe.outcome is None and time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.1)
        if probe.outcome is None:
            raise RuntimeError('executor did not publish a timeout outcome')
        assert probe.outcome.failure_code == 'OBSERVATION_STALE', probe.outcome.failure_code
        assert probe.command_count == 0, probe.command_count
        print('stale generation rejected; outcome=OBSERVATION_STALE; joint commands=0')
    finally:
        executor.terminate()
        executor.wait(timeout=5)
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
