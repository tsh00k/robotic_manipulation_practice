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
Online check of the estimator's ~/initial_box_pose topic (Week 4.1 Stage 5).

Source the workspace under test and use an otherwise unused ROS domain:
    source install/setup.bash
    ROS_DOMAIN_ID=84 /usr/bin/python3 src/mujoco_perception/test/initial_box_probe.py static
    ROS_DOMAIN_ID=84 /usr/bin/python3 src/mujoco_perception/test/initial_box_probe.py episode

Both commands start their own nodes as direct executables (never through ros2 run), log to
/tmp/initial_box_probe_*.log and always reap them.

static   One bridge per random layout (seed 20261007, none of the Stage 3 / Stage 14 sets), the
         arm at HOME, the estimator running. Acceptance, written before the first run:
           S1 the first message of the new generation has frames_averaged == 1 (the window
              restarted at the reset) and state WARMING_UP;
           S2 no MEASURED message has frames_averaged < frames_required;
           S3 the layout is MEASURED within 60 s of wall time, and from the first MEASURED on
              every message of the next 2 s of simulated time is MEASURED;
           S4 every MEASURED message is within 3 mm in x and y, 2 mm in z and 5 degrees in yaw
              (modulo 90) of the bridge's box pose. These are the pixel-quantisation bounds of
              the unit tests; the Stage 2 tolerances (10 mm, 30 degrees) are much looser.
episode  Bridge + estimator + executor with observation_source=vision, three episodes in a
         row, on the legacy scene (no bin; --scene bin adds the bin). Acceptance:
           E1 every episode ends in success with zero retries (the older detector path still
              works after the RGB pairing was removed). Legacy scene only: E1 was first written
              for the bin scene and failed in all three episodes with OBSERVATION_STALE, after
              a complete pick and place, because the older detector answers CANDIDATE_INVALID
              for a box lying in the bin. The build before this change behaves the same, so it
              is not caused by it; judging the box in the bin is Week 4.1 Stage 10-11. On
              --scene bin E1 is reported, not required;
           E2 no message is published for a depth frame whose own bridge sample is ATTACHED;
           E3 the first message of each new generation has frames_averaged == 1;
           E4 after a release the first message has frames_averaged == 1 (the window restarted)
              and every later message in that generation has frames_averaged counting from it.
         Reported, not accepted or rejected: the fraction of the depth frames that are not
         ATTACHED which produced a message (on the new topic and on the older object_pose
         topic; the same count with `--baseline` on the pre-change workspace), and what the detector reports for the box lying in the bin after the release.
The bridge's box pose is used to judge the result and is never given to a node under test.
"""

import argparse
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from manipulation_interfaces.msg import BridgeObservation, EpisodeOutcome, VisionObjectPose
try:
    from manipulation_interfaces.msg import InitialBoxPose
except ImportError:  # a workspace from before Stage 5, used by `episode --baseline`
    InitialBoxPose = None
from manipulation_interfaces.srv import ResetScene
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty

BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=50)
XY_LIMIT_M = 0.003
Z_LIMIT_M = 0.002
YAW_LIMIT_DEG = 5.0
STABLE_S = 2.0
LAYOUT_SEED = 20261007


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def fold90(angle_deg):
    return (angle_deg + 45.0) % 90.0 - 45.0


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class Rig:
    """Bridge + estimator (+ executor), started directly and always reaped."""

    def __init__(self, workspace, bridge_params, with_executor, initial_box=True):
        self.install = Path(workspace) / 'install'
        self.images = {'raw': {}, 'filtered': {}, 'mask': {}}
        self.processes = []
        self.logs = []
        self.observations = []
        self.messages = []
        self.object_poses = []
        self.depth_stamps = []
        self.outcomes = []
        rclpy.init()
        self.node = rclpy.create_node('initial_box_probe')
        self.node.create_subscription(
            BridgeObservation, '/mujoco_bridge/episode_observation',
            self.observations.append, 200)
        self.node.create_subscription(
            VisionObjectPose, '/object_pose_estimator/object_pose',
            self.object_poses.append, 200)
        if initial_box:
            self.node.create_subscription(
                InitialBoxPose, '/object_pose_estimator/initial_box_pose',
                self.messages.append, 200)
        self.node.create_subscription(
            Image, '/mujoco_bridge/camera/depth/image_raw',
            lambda m: self.depth_stamps.append(stamp_ns(m.header.stamp)), BEST_EFFORT)
        for key, topic in (('raw', '/mujoco_bridge/camera/depth/image_raw'),
                           ('filtered', '/object_pose_estimator/debug/filtered_depth'),
                           ('mask', '/object_pose_estimator/debug/robot_mask')):
            self.node.create_subscription(
                Image, topic, lambda m, k=key: self.keep_image(k, m), BEST_EFFORT)
        self.node.create_subscription(
            EpisodeOutcome, '/task_executor/episode_outcome', self.outcomes.append, 10)
        self.start = self.node.create_publisher(Empty, '/task_executor/start_episode', 10)
        self.reset_client = self.node.create_client(
            ResetScene, '/mujoco_bridge/reset_with_generation')
        for _ in range(20):
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if self.node.count_publishers('/clock') != 0:
            raise RuntimeError('a /clock publisher already exists; stop leftover nodes first')

        sim_time = ['--ros-args', '-p', 'use_sim_time:=true']
        bridge_arguments = ['--ros-args']
        for item in bridge_params:
            bridge_arguments += ['-p', item.replace('=', ':=', 1)]
        nodes = [
            ('mujoco_bridge', 'mujoco_bridge_node', bridge_arguments,
             {'LIBGL_ALWAYS_SOFTWARE': '1'}),  # the camera renders offscreen
            ('mujoco_perception', 'object_pose_estimator_node', sim_time,
             {'LIBGL_ALWAYS_SOFTWARE': '1'}),
        ]
        if with_executor:
            nodes.append(('task_executor', 'task_executor_node',
                          sim_time + ['-p', 'observation_source:=vision'], {}))
        for package, name, arguments, env in nodes:
            log = open(f'/tmp/initial_box_probe_{package}.log', 'w')
            self.logs.append(log)
            self.processes.append(subprocess.Popen(
                [str(self.install / package / 'lib' / package / name)] + arguments,
                stdout=log, stderr=subprocess.STDOUT, env=dict(os.environ, **env),
                start_new_session=True))
        topic = '/object_pose_estimator/initial_box_pose' if initial_box else \
            '/object_pose_estimator/object_pose'
        ready = self.spin_until(
            lambda: bool(self.observations) and self.node.count_publishers(topic) > 0, 60)
        clock_publishers = self.node.count_publishers('/clock')
        if not ready or clock_publishers != 1:
            detail = (f'observations={len(self.observations)} {topic} publishers='
                      f'{self.node.count_publishers(topic)} '
                      f'/clock publishers={clock_publishers}')
            self.close()
            raise RuntimeError(
                f'nodes did not come up ({detail}); see /tmp/initial_box_probe_*.log')

    def keep_image(self, key, message):
        """Keep the last 40 images of a topic by stamp (only used by --dump)."""
        store = self.images[key]
        dtype = np.uint8 if message.encoding == 'mono8' else np.float32
        store[stamp_ns(message.header.stamp)] = np.frombuffer(
            bytes(message.data), dtype).reshape(message.height, message.width).copy()
        while len(store) > 40:
            del store[min(store)]

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return predicate()

    def reset(self):
        if not self.reset_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('reset service missing')
        future = self.reset_client.call_async(ResetScene.Request())
        self.spin_until(future.done, 10.0)
        return future.result().generation

    def close(self):
        for process in self.processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
        for process in self.processes:
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for log in self.logs:
            log.close()
        self.spin_until(lambda: self.node.count_publishers('/clock') == 0, 8.0)
        self.node.destroy_node()
        rclpy.shutdown()


def check_pose(message, truth):
    """Return the errors of a MEASURED message against the bridge box pose (mm, mm, deg)."""
    p = truth.object_pose.pose
    dx = (message.position.x - p.position.x) * 1000
    dy = (message.position.y - p.position.y) * 1000
    dz = (message.position.z - p.position.z) * 1000
    dyaw = fold90(math.degrees(message.yaw_rad) - math.degrees(yaw_of(p.orientation)))
    return dx, dy, dz, dyaw


def within_limits(errors):
    dx, dy, dz, dyaw = errors
    return (abs(dx) <= XY_LIMIT_M * 1000 and abs(dy) <= XY_LIMIT_M * 1000 and
            abs(dz) <= Z_LIMIT_M * 1000 and abs(dyaw) <= YAW_LIMIT_DEG)


def make_layouts(count):
    rng = np.random.default_rng(LAYOUT_SEED)
    # y stays below 0.15 so that the box is clear of the bin (y from 0.229), as the bridge
    # requires.
    return [dict(x=float(rng.uniform(0.38, 0.62)), y=float(rng.uniform(-0.22, 0.15)),
                 yaw=float(rng.uniform(0.0, math.pi / 2))) for _ in range(count)]


def run_static(args):
    failures = []
    for index, layout in enumerate(make_layouts(args.count)):
        label = (f'layout {index}: x={layout["x"]:.3f} y={layout["y"]:.3f} '
                 f'yaw={math.degrees(layout["yaw"]):.1f}')
        rig = Rig(args.workspace, [
            'scene.enabled=true', 'enable_rgbd_camera=true', f'scene.box.x={layout["x"]}',
            f'scene.box.y={layout["y"]}', f'scene.box.yaw={layout["yaw"]}'], False)
        try:
            generation = rig.reset()
            first_wall = time.monotonic()
            mine = lambda: [m for m in rig.messages if m.generation == generation]  # noqa: E731
            rig.spin_until(
                lambda: any(m.state == InitialBoxPose.MEASURED for m in mine()),
                args.timeout_s)
            wall_to_measured = time.monotonic() - first_wall
            messages = mine()
            if not messages:
                failures.append(f'{label}: no message for generation {generation}')
                continue
            first_measured = next(
                (m for m in messages if m.state == InitialBoxPose.MEASURED), None)
            if first_measured is None:
                failures.append(f'{label}: S3 not MEASURED within {args.timeout_s} s '
                                f'(last: {messages[-1].reason})')
                continue
            t0 = stamp_ns(first_measured.header.stamp)
            rig.spin_until(
                lambda: bool(mine()) and stamp_ns(mine()[-1].header.stamp) >= t0 + int(
                    STABLE_S * 1e9), 60)
            messages = mine()
            truth = next(o for o in reversed(rig.observations) if o.generation == generation)
            problems = []
            if messages[0].frames_averaged != 1 or messages[0].state != InitialBoxPose.WARMING_UP:
                problems.append(f'S1 first message frames={messages[0].frames_averaged} '
                                f'state={messages[0].state}')
            early = [m for m in messages if m.state == InitialBoxPose.MEASURED and
                     m.frames_averaged < m.frames_required]
            if early:
                problems.append(f'S2 {len(early)} MEASURED messages before the window was full')
            stable = [m for m in messages if stamp_ns(m.header.stamp) >= t0]
            not_measured = [m for m in stable if m.state != InitialBoxPose.MEASURED]
            if not_measured:
                problems.append(f'S3 {len(not_measured)} of {len(stable)} messages after the '
                                f'first MEASURED were not measured ({not_measured[0].reason})')
            if args.dump:
                Path(args.dump).mkdir(parents=True, exist_ok=True)
                p = truth.object_pose.pose
                np.savez(
                    Path(args.dump) / f'layout_{index}.npz', stamp=t0,
                    raw=rig.images['raw'].get(t0), filtered=rig.images['filtered'].get(t0),
                    mask=rig.images['mask'].get(t0),
                    window_stamps=sorted(k for k in rig.images['raw'] if k <= t0)[-10:],
                    window_raw=[rig.images['raw'][k] for k in sorted(
                        k for k in rig.images['raw'] if k <= t0)[-10:]],
                    window_filtered=[rig.images['filtered'].get(k) for k in sorted(
                        k for k in rig.images['raw'] if k <= t0)[-10:]],
                    truth=[p.position.x, p.position.y, p.position.z,
                           math.degrees(yaw_of(p.orientation))],
                    measured=[first_measured.position.x, first_measured.position.y,
                              first_measured.position.z, math.degrees(first_measured.yaw_rad)],
                    # (stamp, x, y, z, yaw) of the bridge's box and of every message, to see
                    # whether the box was still moving while the window filled
                    truth_series=[
                        [stamp_ns(o.joint_state.header.stamp), o.object_pose.pose.position.x,
                         o.object_pose.pose.position.y, o.object_pose.pose.position.z,
                         math.degrees(yaw_of(o.object_pose.pose.orientation))]
                        for o in rig.observations if o.generation == generation],
                    message_series=[
                        [stamp_ns(m.header.stamp), m.state, m.position.x, m.position.y,
                         m.position.z, math.degrees(m.yaw_rad)] for m in messages])
            errors = np.array([check_pose(m, truth) for m in stable
                               if m.state == InitialBoxPose.MEASURED])
            bad = [e for e in errors if not within_limits(e)]
            if bad:
                problems.append(f'S4 {len(bad)} MEASURED messages outside the limits, '
                                f'first {np.round(bad[0], 2).tolist()}')
            sim_to_measured = (t0 - stamp_ns(messages[0].header.stamp)) / 1e9
            worst = np.abs(errors).max(axis=0) if len(errors) else [float('nan')] * 4
            print(f'{label}: MEASURED after {first_measured.frames_averaged} frames, '
                  f'{sim_to_measured:.2f} s sim / {wall_to_measured:.1f} s wall; {len(stable)} '
                  f'stable messages, worst |dx| {worst[0]:.2f} |dy| {worst[1]:.2f} '
                  f'|dz| {worst[2]:.2f} mm, |dyaw| {worst[3]:.2f} deg; '
                  f'{"OK" if not problems else "FAIL"}', flush=True)
            failures += [f'{label}: {p}' for p in problems]
        finally:
            rig.close()
    print('\nSTATIC ACCEPTANCE:', 'PASS' if not failures else 'FAIL')
    for failure in failures:
        print('  ', failure)
    return 0 if not failures else 1


def run_episode(args):
    bridge_params = ['enable_rgbd_camera=true']
    if args.scene == 'bin':
        bridge_params.append('scene.enabled=true')
    rig = Rig(args.workspace, bridge_params, True, initial_box=not args.baseline)
    failures = []
    try:
        if not rig.spin_until(lambda: rig.start.get_subscription_count() > 0, 30):
            raise RuntimeError('executor did not come up')
        for episode in range(args.episodes):
            rig.outcomes.clear()
            rig.start.publish(Empty())
            if not rig.spin_until(lambda: bool(rig.outcomes), 240):
                failures.append(f'episode {episode}: no outcome within 240 s')
                break
            rig.spin_until(lambda: False, 4.0)  # the release is followed by a few more frames
            outcome = rig.outcomes[-1]
            if (not outcome.success or outcome.retries != 0) and args.scene == 'legacy':
                failures.append(f'E1 episode {episode}: success={outcome.success} '
                                f'retries={outcome.retries} failure={outcome.failure_code}')
            generation = max(o.generation for o in rig.observations)
            sample_state = {stamp_ns(o.joint_state.header.stamp): o.attachment_state
                            for o in rig.observations if o.generation == generation}
            messages = [m for m in rig.messages if m.generation == generation]
            depth_frames = [s for s in rig.depth_stamps if s in sample_state]
            free_frames = [s for s in depth_frames
                           if sample_state[s] != BridgeObservation.ATTACHMENT_ATTACHED]
            old_path = [m for m in rig.object_poses if m.generation == generation]
            old_note = (f'older object_pose topic: {len(old_path)} messages for '
                        f'{len(free_frames)} depth frames not ATTACHED '
                        f'({100 * len(old_path) / max(len(free_frames), 1):.0f} %)')
            if args.baseline:
                print(f'episode {episode}: success={outcome.success} retries={outcome.retries} '
                      f'failure={outcome.failure_code}; {old_note}', flush=True)
                continue
            if not messages:
                failures.append(f'episode {episode}: no initial box messages')
                continue
            attached_stamps = [k for k, s in sample_state.items()
                               if s == BridgeObservation.ATTACHMENT_ATTACHED]
            leaked = [m for m in messages
                      if sample_state.get(stamp_ns(m.header.stamp)) ==
                      BridgeObservation.ATTACHMENT_ATTACHED]
            if leaked:
                failures.append(f'E2 episode {episode}: {len(leaked)} messages for ATTACHED '
                                f'samples, first frames_averaged={leaked[0].frames_averaged}')
            if messages[0].frames_averaged != 1:
                failures.append(f'E3 episode {episode}: first message frames_averaged='
                                f'{messages[0].frames_averaged}')
            release_note = 'no attached period'
            if attached_stamps:
                release = max(attached_stamps)
                after = [m for m in messages if stamp_ns(m.header.stamp) > release]
                if not after:
                    failures.append(f'E4 episode {episode}: no message after the release')
                else:
                    counts = [m.frames_averaged for m in after]
                    if counts[0] != 1:
                        failures.append(f'E4 episode {episode}: first message after release '
                                        f'has frames_averaged={counts[0]}')
                    last = after[-1]
                    release_note = (
                        f'after release {len(after)} messages, last state {last.state} '
                        f'reason {last.reason or "-"} frames={last.frames_averaged}, box at '
                        f'({last.position.x:.3f}, {last.position.y:.3f}, {last.position.z:.3f})')
            print(f'episode {episode}: success={outcome.success} retries={outcome.retries}; '
                  f'initial_box_pose: {len(messages)} messages for {len(free_frames)} depth '
                  f'frames not ATTACHED ({100 * len(messages) / max(len(free_frames), 1):.0f} %), '
                  f'{len(depth_frames) - len(free_frames)} ATTACHED frames skipped; {old_note}; '
                  f'{release_note}', flush=True)
    finally:
        rig.close()
    if args.baseline:
        print('\nBASELINE: outcomes only, nothing judged')
        return 0
    print('\nEPISODE ACCEPTANCE:', 'PASS' if not failures else 'FAIL')
    for failure in failures:
        print('  ', failure)
    return 0 if not failures else 1


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command', choices=['static', 'episode'])
    parser.add_argument('--workspace', default=str(Path(__file__).resolve().parents[3]))
    parser.add_argument('--count', type=int, default=6)
    parser.add_argument('--episodes', type=int, default=3)
    parser.add_argument('--baseline', action='store_true',
                        help='episode: a workspace without the initial box topic; only report '
                             'the outcomes (use --workspace)')
    parser.add_argument('--scene', choices=['legacy', 'bin'], default='legacy',
                        help='episode: the simulator scene (bin = scene.enabled with the bin)')
    parser.add_argument('--timeout-s', type=float, default=60.0)
    parser.add_argument('--dump', help='static: save the raw/masked frame at the first MEASURED')
    args = parser.parse_args()
    return run_static(args) if args.command == 'static' else run_episode(args)


if __name__ == '__main__':
    sys.exit(main())
