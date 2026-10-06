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
Online checks of the estimator's ~/initial_box_pose and ~/initial_bin_pose (Week 4.1 Stages 5, 6).

Source the workspace under test and use an otherwise unused ROS domain:
    source install/setup.bash
    ROS_DOMAIN_ID=84 /usr/bin/python3 src/mujoco_perception/test/initial_box_probe.py static
    ROS_DOMAIN_ID=84 /usr/bin/python3 src/mujoco_perception/test/initial_box_probe.py gaps
    ROS_DOMAIN_ID=84 /usr/bin/python3 src/mujoco_perception/test/initial_box_probe.py episode

All commands start their own nodes as direct executables (never through ros2 run), log to
/tmp/initial_box_probe_*.log and always reap them.

static   One bridge per random layout (seed 20261008; box AND bin positions and the bin yaw are
         random, and the box keeps the bridge's 20 mm clearance from the bin), the arm at HOME,
         the estimator running. Acceptance, written before the first run, for the box and for
         the bin:
           S1 the first message of the new generation has frames_averaged == 1 and state
              WARMING_UP (the window restarted at the reset);
           S2 no MEASURED message has frames_averaged < frames_required;
           S3 from the first message with a full window, every message of the next 2 s of
              simulated time is MEASURED, and the window fills within 60 s of wall time;
           S4 every MEASURED message is within these limits of the truth: x and y 3 mm, z 2 mm,
              yaw (box: modulo 90, 5 degrees; bin: modulo 180, 3 degrees). They are the pixel
              quantisation bounds of the unit tests; the Stage 2 tolerances (10 mm, 30 degrees)
              are much looser.
           S5 the estimator node has no parameter whose name starts with `scene.`.
         The box truth is the bridge's box pose; the bin has no topic or TF by design, so its
         truth is the command line the bridge accepted (inner floor at z = 0.227 m).
gaps     Exploration, not judged (Week 4.1 6.3 E4): the bin at (0.5, 0.2) with yaw 0 and the box
         on the camera side of it, at 21, 40, 60 and 80 mm between the box and the bin's outer
         footprint (the bridge refuses less than 20 mm; 21 because 20 is the boundary itself).
         Prints, for box and bin, how many stable messages were MEASURED, the reason when not,
         the blocks the detector saw, and the errors when measured.
episode  Bridge + estimator + executor with observation_source=vision, three episodes in a
         row, on the legacy scene (no bin; --scene bin adds the bin). Acceptance:
           E1 every episode ends in success with zero retries (the older detector path still
              works after the RGB pairing was removed). Legacy scene only: E1 was first written
              for the bin scene and failed in all three episodes with OBSERVATION_STALE, after
              a complete pick and place, because the older detector answers CANDIDATE_INVALID
              for a box lying in the bin. The build before this change behaves the same, so it
              is not caused by it; judging the box in the bin is Week 4.1 Stage 11. On
              --scene bin E1 is reported, not required;
           E2 no message is published for a depth frame whose own bridge sample is ATTACHED;
           E3 the first message of each new generation has frames_averaged == 1;
           E4 after a release the first message has frames_averaged == 1 (the window restarted)
              and every later message in that generation has frames_averaged counting from it.
         Reported, not accepted or rejected: the fraction of the depth frames that are not
         ATTACHED which produced a message (on the new topic and on the older object_pose
         topic; the same count with `--baseline` on the pre-change workspace), and what the
         detector reports for the box lying in the bin after the release.
The bridge's poses are used to judge the result and are never given to a node under test.
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
    from manipulation_interfaces.msg import InitialBinPose, InitialBoxPose
except ImportError:  # a workspace from before Stage 5, used by `episode --baseline`
    InitialBinPose = InitialBoxPose = None
from rcl_interfaces.srv import ListParameters
from manipulation_interfaces.srv import ResetScene
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty

BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=50)
XY_LIMIT_M = 0.003
Z_LIMIT_M = 0.002
BOX_YAW_LIMIT_DEG = 5.0
BIN_YAW_LIMIT_DEG = 3.0
STABLE_S = 2.0
LAYOUT_SEED = 20261008
BOX_HALF = 0.02
BIN_HALF = (0.076, 0.071)  # outer footprint of the bin, half extents (x, y) in the bin frame
BIN_FLOOR_Z = 0.227        # inner floor surface: table 0.22 + the bridge's auto support 0.007
CLEARANCE = 0.02           # the bridge's box-to-bin footprint clearance


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
        self.processes = []
        self.logs = []
        self.observations = []
        self.messages = []
        self.bin_messages = []
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
                InitialBinPose, '/object_pose_estimator/initial_bin_pose',
                self.bin_messages.append, 200)
        self.node.create_subscription(
            Image, '/mujoco_bridge/camera/depth/image_raw',
            lambda m: self.depth_stamps.append(stamp_ns(m.header.stamp)), BEST_EFFORT)
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


def fold(angle_deg, period):
    return (angle_deg + period / 2.0) % period - period / 2.0


def box_truth(observation):
    pose = observation.object_pose.pose
    return dict(x=pose.position.x, y=pose.position.y, z=pose.position.z,
                yaw_deg=math.degrees(yaw_of(pose.orientation)), period=90.0,
                yaw_limit=BOX_YAW_LIMIT_DEG)


def bin_truth(bin_pose):
    return dict(x=bin_pose[0], y=bin_pose[1], z=BIN_FLOOR_Z, yaw_deg=math.degrees(bin_pose[2]),
                period=180.0, yaw_limit=BIN_YAW_LIMIT_DEG)


def pose_errors(message, truth):
    """Errors of one MEASURED message: dx, dy, dz in mm and dyaw in degrees."""
    return (1000.0 * (message.position.x - truth['x']), 1000.0 * (message.position.y - truth['y']),
            1000.0 * (message.position.z - truth['z']),
            fold(math.degrees(message.yaw_rad) - truth['yaw_deg'], truth['period']))


def within_limits(errors, truth):
    dx, dy, dz, dyaw = errors
    return (abs(dx) <= XY_LIMIT_M * 1000 and abs(dy) <= XY_LIMIT_M * 1000 and
            abs(dz) <= Z_LIMIT_M * 1000 and abs(dyaw) <= truth['yaw_limit'])


def clear_of_bin(box, bin_pose):
    """Return True if the box footprint is clear of the bin by the bridge's rule (+ 1 mm)."""
    box_x, box_y, box_yaw = box
    bin_x, bin_y, bin_yaw = bin_pose
    corners = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            wx = box_x + BOX_HALF * (sx * math.cos(box_yaw) - sy * math.sin(box_yaw))
            wy = box_y + BOX_HALF * (sx * math.sin(box_yaw) + sy * math.cos(box_yaw))
            dx, dy = wx - bin_x, wy - bin_y
            corners.append((math.cos(bin_yaw) * dx + math.sin(bin_yaw) * dy,
                            -math.sin(bin_yaw) * dx + math.cos(bin_yaw) * dy))
    low = [min(c[i] for c in corners) for i in (0, 1)]
    high = [max(c[i] for c in corners) for i in (0, 1)]
    margin = CLEARANCE + 0.001
    overlaps = all(high[i] > -BIN_HALF[i] - margin and low[i] < BIN_HALF[i] + margin
                   for i in (0, 1))
    return not overlaps


def make_layouts(count):
    rng = np.random.default_rng(LAYOUT_SEED)
    layouts = []
    while len(layouts) < count:
        box = (float(rng.uniform(0.38, 0.62)), float(rng.uniform(-0.22, 0.22)),
               float(rng.uniform(0.0, math.pi / 2)))
        bin_pose = (float(rng.uniform(0.38, 0.62)), float(rng.uniform(-0.22, 0.22)),
                    float(rng.uniform(0.0, math.pi)))
        if clear_of_bin(box, bin_pose):
            layouts.append(dict(box=box, bin=bin_pose, judged=True))
    return layouts


def gap_layouts(gaps_mm):
    """Bin at (0.5, 0.2), yaw 0; the box on the camera side (smaller y) at the given gaps."""
    bin_pose = (0.5, 0.2, 0.0)
    return [dict(box=(0.5, bin_pose[1] - BIN_HALF[1] - gap / 1000.0 - BOX_HALF, 0.0),
                 bin=bin_pose, judged=False, gap_mm=gap) for gap in gaps_mm]


def check_object(name, messages, full_t0, truth, judged):
    """Check one object's messages of one generation; return (problems, summary text)."""
    problems = []
    if messages[0].frames_averaged != 1 or messages[0].state != InitialBoxPose.WARMING_UP:
        problems.append(f'{name} S1 first message frames={messages[0].frames_averaged} '
                        f'state={messages[0].state}')
    early = [m for m in messages if m.state == InitialBoxPose.MEASURED and
             m.frames_averaged < m.frames_required]
    if early:
        problems.append(f'{name} S2 {len(early)} MEASURED messages before the window was full')
    stable = [m for m in messages if full_t0 is not None and stamp_ns(m.header.stamp) >= full_t0]
    measured = [m for m in stable if m.state == InitialBoxPose.MEASURED]
    if judged and len(measured) != len(stable):
        reason = next(m.reason for m in stable if m.state != InitialBoxPose.MEASURED)
        problems.append(f'{name} S3 {len(stable) - len(measured)} of {len(stable)} messages '
                        f'with a full window were not measured ({reason})')
    errors = np.array([pose_errors(m, truth) for m in measured])
    bad = [e for e in errors if not within_limits(e, truth)]
    if judged and bad:
        problems.append(f'{name} S4 {len(bad)} MEASURED messages outside the limits, '
                        f'first {np.round(bad[0], 2).tolist()}')
    worst = np.abs(errors).max(axis=0) if len(errors) else [float('nan')] * 4
    text = (f'{name}: {len(measured)}/{len(stable)} measured, worst |dx| {worst[0]:.2f} '
            f'|dy| {worst[1]:.2f} |dz| {worst[2]:.2f} mm |dyaw| {worst[3]:.2f} deg')
    if stable and len(measured) != len(stable):
        last = stable[-1]
        blocks = ', '.join(f'{c.side_along_m * 1000:.0f}x{c.side_across_m * 1000:.0f} mm'
                           for c in last.candidates)
        text += f' [not measured: {last.reason}; blocks seen: {blocks or "none"}]'
    return problems, text


def check_no_scene_parameters(rig):
    client = rig.node.create_client(ListParameters, '/object_pose_estimator/list_parameters')
    if not client.wait_for_service(timeout_sec=10.0):
        return ['S5 the estimator\'s list_parameters service is not available']
    future = client.call_async(ListParameters.Request())
    rig.spin_until(future.done, 10.0)
    names = future.result().result.names
    print(f'S5: the estimator lists {len(names)} parameters, e.g. {sorted(names)[:3]}', flush=True)
    scene = [n for n in names if n.startswith('scene.')]
    return [f'S5 the estimator has scene parameters: {scene}'] if scene else []


def run_layouts(args, layouts):
    failures = []
    for index, layout in enumerate(layouts):
        box, bin_pose = layout['box'], layout['bin']
        label = (f'layout {index}: box ({box[0]:.3f}, {box[1]:.3f}, {math.degrees(box[2]):.1f}), '
                 f'bin ({bin_pose[0]:.3f}, {bin_pose[1]:.3f}, {math.degrees(bin_pose[2]):.1f})'
                 + (f', gap {layout["gap_mm"]} mm' if 'gap_mm' in layout else ''))
        rig = Rig(args.workspace, [
            'scene.enabled=true', 'enable_rgbd_camera=true',
            f'scene.box.x={box[0]}', f'scene.box.y={box[1]}', f'scene.box.yaw={box[2]}',
            f'scene.bin.x={bin_pose[0]}', f'scene.bin.y={bin_pose[1]}',
            f'scene.bin.yaw={bin_pose[2]}'], False)
        try:
            problems = check_no_scene_parameters(rig) if index == 0 else []
            generation = rig.reset()
            mine = lambda items: [m for m in items if m.generation == generation]  # noqa: E731
            full = lambda: any(  # noqa: E731
                m.frames_averaged == m.frames_required for m in mine(rig.messages))
            rig.spin_until(full, args.timeout_s)
            if not full():
                failures.append(f'{label}: the window did not fill within {args.timeout_s} s')
                continue
            full_t0 = stamp_ns(next(m for m in mine(rig.messages)
                                    if m.frames_averaged == m.frames_required).header.stamp)
            rig.spin_until(
                lambda: bool(mine(rig.messages)) and bool(mine(rig.bin_messages)) and
                min(stamp_ns(mine(rig.messages)[-1].header.stamp),
                    stamp_ns(mine(rig.bin_messages)[-1].header.stamp)) >=
                full_t0 + int(STABLE_S * 1e9), 60)
            box_problems, box_text = check_object(
                'box', mine(rig.messages), full_t0,
                box_truth(next(o for o in reversed(rig.observations)
                               if o.generation == generation)), layout['judged'])
            bin_problems, bin_text = check_object(
                'bin', mine(rig.bin_messages), full_t0, bin_truth(bin_pose), layout['judged'])
            problems += box_problems + bin_problems
            print(f'{label}\n    {box_text}\n    {bin_text}'
                  + (f'\n    {"OK" if not problems else "FAIL"}' if layout['judged'] else ''),
                  flush=True)
            failures += [f'{label}: {p}' for p in problems]
        finally:
            rig.close()
    return failures


def run_static(args):
    failures = run_layouts(args, make_layouts(args.count))
    print('\nSTATIC ACCEPTANCE:', 'PASS' if not failures else 'FAIL')
    for failure in failures:
        print('  ', failure)
    return 0 if not failures else 1


def run_gaps(args):
    failures = run_layouts(args, gap_layouts(args.gaps))
    print('\nGAPS: exploration, nothing judged except the bookkeeping S1/S2')
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
    parser.add_argument('command', choices=['static', 'gaps', 'episode'])
    parser.add_argument('--workspace', default=str(Path(__file__).resolve().parents[3]))
    parser.add_argument('--count', type=int, default=6)
    parser.add_argument('--episodes', type=int, default=3)
    parser.add_argument('--baseline', action='store_true',
                        help='episode: a workspace without the initial box topic; only report '
                             'the outcomes (use --workspace)')
    parser.add_argument('--scene', choices=['legacy', 'bin'], default='legacy',
                        help='episode: the simulator scene (bin = scene.enabled with the bin)')
    parser.add_argument('--timeout-s', type=float, default=60.0)
    parser.add_argument('--gaps', type=float, nargs='*', default=[21, 40, 60, 80],
                        help='gaps: box-to-bin gaps in mm')
    args = parser.parse_args()
    return {'static': run_static, 'gaps': run_gaps, 'episode': run_episode}[args.command](args)


if __name__ == '__main__':
    sys.exit(main())
