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
latch    Week 4.1 Stage 7: bridge + estimator + executor (vision) with the box AND the bin in the
         scene (the first random layout), two episodes started one after the other, and a third
         run in a scene without a bin whose executor insists on one. Assertions, written before
         the first run:
           A1 for each generation, no joint command is published between the first latch
              status of that generation and its LATCHED;
           A2 the latched box and bin are within the S4 limits of the truth, and the TCP x, y
              of the GRASP target in the executor log are within 1 mm of the latched box;
           A3 (no bin in the scene, place.into_bin:=true, latch.timeout_s:=8) the episode
              ends with VISION_LATCH_TIMEOUT, failure layer "perception", a reason that names
              the bin and not the box, and no joint command was published at all;
           A4 the second episode latches under a new generation, from estimates that belong to
              that generation (their sequence lies in the range of the bridge samples of that
              generation), again with no command before LATCHED.
place    Week 4.1 Stage 8. `--source oracle|vision`, the first `--count` random layouts (box AND
         bin random), one bridge and one episode per layout, place.into_bin:=true. Checks,
         written before the first run:
           B1 (oracle) PREPLACE and PLACE TCP x, y within 1 mm of the true bin, PLACE z =
              0.227 + 0.07 m (+-1 mm); the episode succeeds with zero retries and the box ends
              inside the bin (all four corners inside the inner walls, centre below 0.252 m,
              judged on the bridge's true box pose). A layout ending in IK_FAILED is recorded,
              not failed, but at least one layout must succeed.
           B2 (vision) the same targets, against the LATCHED bin (x, y within 1 mm, z = latched
              floor + 0.07 m +-1 mm), and the box inside the bin after the release; the
              episode's own outcome is reported only (VERIFY is expected to end
              OBSERVATION_STALE until Stage 11).
           B3 (reported) at the release: the fingertip (8.9 mm below the TCP) above the wall
              top (0.239 m), and the box bottom above the inner floor (0.227 m). The release
              is the sample right after the LAST attached run; it was first the first
              attached-to-not-attached change, which caught an attachment change during the
              grasp (Week 4.1 8.3).
         `--scene legacy`: no bin, place.into_bin false, three episodes in one run (B4): each
         succeeds with zero retries and the PLACE target is (0.5, 0.3, 0.29).
         `--align-check` (Week 4.1 Stage 9) judges instead, per episode: C1/C2 the tool
         rotation of GRASP equals the box yaw folded to [-45, 45) (truth for oracle, latched
         for vision) within 1 degree and PLACE keeps it within 0.1 degree; one attached run
         before the final release; the width at the attach at most 45 mm; C4 (legacy) the
         GRASP rotation within 1 degree of 0. Success and the box in the bin are reported.
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
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from manipulation_interfaces.msg import (
    BridgeObservation, EpisodeOutcome, InitialPoseLatch, VisionObjectPose)
try:
    from manipulation_interfaces.msg import InitialBinPose, InitialBoxPose
except ImportError:  # a workspace from before Stage 5, used by `episode --baseline`
    InitialBinPose = InitialBoxPose = None
from rcl_interfaces.srv import ListParameters
from manipulation_interfaces.srv import ResetScene
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory

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

    def __init__(self, workspace, bridge_params, with_executor, initial_box=True,
                 executor_params=(), source='vision'):
        self.install = Path(workspace) / 'install'
        self.processes = []
        self.logs = []
        self.observations = []
        self.messages = []
        self.events = []  # ('cmd',) and ('latch', message) in the order they arrived
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
            JointTrajectory, '/mujoco_bridge/joint_command',
            lambda m: self.events.append(('cmd',)), 50)
        self.node.create_subscription(
            InitialPoseLatch, '/task_executor/initial_pose_latch',
            lambda m: self.events.append(('latch', m)), 200)
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
            executor_arguments = sim_time + ['-p', f'observation_source:={source}']
            for item in executor_params:
                executor_arguments += ['-p', item.replace('=', ':=', 1)]
            nodes.append(('task_executor', 'task_executor_node', executor_arguments, {}))
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


GRASP_LINE = re.compile(
    r'phase GRASP target_frame=world tcp_xyz=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]')
EXECUTOR_LOG = Path('/tmp/initial_box_probe_task_executor.log')


def grasp_targets():
    """Return the TCP targets of every GRASP phase the executor has logged so far."""
    if not EXECUTOR_LOG.exists():
        return []
    return [tuple(float(v) for v in m.groups())
            for m in GRASP_LINE.finditer(EXECUTOR_LOG.read_text(errors='replace'))]


def statuses(rig, generation):
    return [(i, e[1]) for i, e in enumerate(rig.events)
            if e[0] == 'latch' and e[1].generation == generation]


def check_generation(rig, generation, truth_box, truth_bin, label):
    """A1, A2 (latched poses against truth) and A4's generation bookkeeping for one generation."""
    problems = []
    found = statuses(rig, generation)
    latched = [(i, m) for i, m in found if m.state == InitialPoseLatch.LATCHED]
    if not latched:
        last = found[-1][1].box.status if found else 'none'
        return [f'{label}: never LATCHED (last status: {last})']
    first_status, first_latched = found[0][0], latched[0][0]
    commands = [i for i in range(first_status, first_latched) if rig.events[i][0] == 'cmd']
    if commands:
        problems.append(f'{label} A1 {len(commands)} joint commands between the first status and '
                        f'LATCHED')
    message = latched[0][1]
    for name, pose, truth in (('box', message.box, truth_box), ('bin', message.bin, truth_bin)):
        errors = pose_errors(pose, truth)
        if not within_limits(errors, truth):
            problems.append(f'{label} A2 latched {name} outside the limits: '
                            f'{[round(e, 2) for e in errors]}')
    own = [o.sample_sequence for o in rig.observations if o.generation == generation]
    for name, pose in (('box', message.box), ('bin', message.bin)):
        if not (min(own) <= pose.source_sequence <= max(own)):
            problems.append(f'{label} A4 the latched {name} estimate has sequence '
                            f'{pose.source_sequence}, outside this generation\'s samples '
                            f'{min(own)}..{max(own)}')
    box_msg, bin_msg = message.box, message.bin
    print(f'    {label}: LATCHED after {len(found)} status messages; box spread '
          f'{box_msg.position_spread_m * 1000:.2f} mm {math.degrees(box_msg.yaw_spread_rad):.2f} '
          f'deg, bin spread {bin_msg.position_spread_m * 1000:.2f} mm '
          f'{math.degrees(bin_msg.yaw_spread_rad):.2f} deg; commands before LATCHED: '
          f'{len(commands)}', flush=True)
    return problems


def run_latch(args):
    failures = []
    layout = make_layouts(1)[0]
    box, bin_pose = layout['box'], layout['bin']
    print(f'layout: box ({box[0]:.3f}, {box[1]:.3f}, {math.degrees(box[2]):.1f}), bin '
          f'({bin_pose[0]:.3f}, {bin_pose[1]:.3f}, {math.degrees(bin_pose[2]):.1f})', flush=True)
    if EXECUTOR_LOG.exists():
        EXECUTOR_LOG.unlink()
    rig = Rig(args.workspace, [
        'scene.enabled=true', 'enable_rgbd_camera=true', f'scene.box.x={box[0]}',
        f'scene.box.y={box[1]}', f'scene.box.yaw={box[2]}', f'scene.bin.x={bin_pose[0]}',
        f'scene.bin.y={bin_pose[1]}', f'scene.bin.yaw={bin_pose[2]}'], True,
        executor_params=['place.into_bin=true'])
    try:
        if not rig.spin_until(lambda: rig.start.get_subscription_count() > 0, 30):
            raise RuntimeError('executor did not come up')
        generations = []
        for episode in (1, 2):
            rig.start.publish(Empty())
            ok = rig.spin_until(lambda: len(grasp_targets()) >= episode, 180)
            generation = max(o.generation for o in rig.observations)
            generations.append(generation)
            label = f'episode {episode} (generation {generation})'
            if not ok:
                failures.append(f'{label}: no GRASP phase within 180 s')
                continue
            first = next(o for o in rig.observations if o.generation == generation)
            problems = check_generation(rig, generation, box_truth(first), bin_truth(bin_pose),
                                        label)
            latched = [m for _, m in statuses(rig, generation)
                       if m.state == InitialPoseLatch.LATCHED]
            target = grasp_targets()[episode - 1]
            if latched:
                miss = 1000.0 * max(abs(target[0] - latched[0].box.position.x),
                                    abs(target[1] - latched[0].box.position.y))
                print(f'    {label}: GRASP target ({target[0]:.4f}, {target[1]:.4f}) differs '
                      f'from the latched box by {miss:.3f} mm', flush=True)
                if miss > 1.0:
                    problems.append(f'{label} A2 the GRASP target differs from the latched box '
                                    f'by {miss:.2f} mm')
            failures += problems
        if len(set(generations)) != 2:
            failures.append(f'A4 the two episodes used generations {generations}')
    finally:
        rig.close()

    print('no-bin scene, place.into_bin:=true, latch.timeout_s:=8', flush=True)
    rig = Rig(args.workspace, ['enable_rgbd_camera=true'], True,
              executor_params=['place.into_bin=true', 'latch.timeout_s=8.0'])
    try:
        if not rig.spin_until(lambda: rig.start.get_subscription_count() > 0, 30):
            raise RuntimeError('executor did not come up')
        rig.start.publish(Empty())
        if not rig.spin_until(lambda: bool(rig.outcomes), 60):
            failures.append('A3 no outcome within 60 s')
        else:
            outcome = rig.outcomes[-1]
            commands = sum(1 for e in rig.events if e[0] == 'cmd')
            print(f'    outcome: success={outcome.success} failure_code={outcome.failure_code} '
                  f'layer={outcome.observation_failure_layer} '
                  f'reason="{outcome.observation_failure_reason}"; joint commands: {commands}',
                  flush=True)
            if outcome.success or outcome.failure_code != 'VISION_LATCH_TIMEOUT':
                failures.append(f'A3 failure code {outcome.failure_code}')
            if outcome.observation_failure_layer != 'perception':
                failures.append(f'A3 failure layer {outcome.observation_failure_layer}')
            reason = outcome.observation_failure_reason
            if not reason.startswith('BIN:WAITING:') or 'BOX:' in reason:
                failures.append(f'A3 the reason should name the bin only: "{reason}"')
            if commands:
                failures.append(f'A3 {commands} joint commands were published')
    finally:
        rig.close()
    print('\nLATCH ACCEPTANCE:', 'PASS' if not failures else 'FAIL')
    for failure in failures:
        print('  ', failure)
    return 0 if not failures else 1


PHASE_TARGET = re.compile(
    r'phase (PREPLACE|PLACE) target_frame=world tcp_xyz=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]')
PHASE_ROTATION = re.compile(
    r'phase (GRASP|PLACE) target_frame=world tcp_xyz=\[[^\]]*\] '
    r'tcp_qwxyz=\[([-\d.]+) ([-\d.]+) ([-\d.]+) ([-\d.]+)\]')


def tool_yaw_deg(qw, qx, qy, qz):
    """Rotation about world z of a TCP orientation relative to the downward reference."""
    # Reference: pi about (1, 1, 0) / sqrt(2), i.e. q0 = (0, 1/sqrt2, 1/sqrt2, 0).
    # q_rel = q * conj(q0); for a pure rotation about z only w and z of q_rel are non-zero.
    a = 1.0 / math.sqrt(2.0)
    w = qx * a + qy * a
    z = qy * a - qx * a  # Rz(t) * q0 = (0, a(cos - sin), a(cos + sin), 0) with t/2 angles
    return math.degrees(2.0 * math.atan2(z, w))


def logged_rotations():
    """Return [(phase, tool yaw in degrees)] of the GRASP and PLACE targets in the log."""
    if not EXECUTOR_LOG.exists():
        return []
    return [(m.group(1), tool_yaw_deg(*(float(v) for v in m.groups()[1:])))
            for m in PHASE_ROTATION.finditer(EXECUTOR_LOG.read_text(errors='replace'))]


def attach_runs_and_width(observations):
    """Return (attached runs before the last release, gripper width at the first attach)."""
    runs, width, previous = 0, None, None
    for o in observations:
        attached = o.attachment_state == BridgeObservation.ATTACHMENT_ATTACHED
        if attached and previous is not True:
            runs += 1
            if width is None:
                names = list(o.joint_state.name)
                width = sum(o.joint_state.position[names.index(n)]
                            for n in ('finger_joint1', 'finger_joint2'))
        previous = attached
    return runs, width


BIN_INNER = (0.070, 0.065)  # inner faces of the bin walls, half extents in the bin frame
ON_FLOOR_Z_MAX = 0.252      # box centre above this rests on a wall rim, not on the floor
FINGERTIP_BELOW_TCP = 0.0089
WALL_TOP_Z = 0.239


def phase_targets():
    """Return [(phase, x, y, z)] of the PREPLACE and PLACE targets the executor logged."""
    if not EXECUTOR_LOG.exists():
        return []
    return [(m.group(1), float(m.group(2)), float(m.group(3)), float(m.group(4)))
            for m in PHASE_TARGET.finditer(EXECUTOR_LOG.read_text(errors='replace'))]


def box_in_bin(observation, bin_pose):
    """Return True if all four box corners lie inside the bin's inner walls, on its floor."""
    pose = observation.object_pose.pose
    yaw = yaw_of(pose.orientation)
    bx, by, byaw = bin_pose
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx = pose.position.x + BOX_HALF * (sx * math.cos(yaw) - sy * math.sin(yaw))
            cy = pose.position.y + BOX_HALF * (sx * math.sin(yaw) + sy * math.cos(yaw))
            lx = math.cos(byaw) * (cx - bx) + math.sin(byaw) * (cy - by)
            ly = -math.sin(byaw) * (cx - bx) + math.cos(byaw) * (cy - by)
            if abs(lx) > BIN_INNER[0] or abs(ly) > BIN_INNER[1]:
                return False
    return pose.position.z < ON_FLOOR_Z_MAX


def release_geometry(observations):
    """
    Return the fingertip margin and box bottom height at the end of the LAST attached run.

    Also returns the attachment states in order, run-length encoded, because the first run
    used the first attached-to-not-attached change and caught a change during the grasp.
    """
    runs = []
    for o in observations:
        if not runs or runs[-1][0] != o.attachment_state:
            runs.append([o.attachment_state, 0])
        runs[-1][1] += 1
    release = None
    for previous, o in zip(observations, observations[1:]):
        if (previous.attachment_state == BridgeObservation.ATTACHMENT_ATTACHED and
                o.attachment_state != BridgeObservation.ATTACHMENT_ATTACHED):
            release = o
    if release is None:
        return None
    tcp_z = release.world_to_hand_tcp.transform.translation.z
    names = {BridgeObservation.ATTACHMENT_NOT_ATTACHED: 'N',
             BridgeObservation.ATTACHMENT_ATTACHED: 'A',
             BridgeObservation.ATTACHMENT_RELEASED: 'R'}
    return ((tcp_z - FINGERTIP_BELOW_TCP - WALL_TOP_Z) * 1000.0,
            (release.object_pose.pose.position.z - BOX_HALF - BIN_FLOOR_Z) * 1000.0,
            ' '.join(f'{names.get(state, state)}x{count}' for state, count in runs))


def run_place(args):
    if args.scene == 'legacy':
        return run_place_legacy(args)
    failures, successes = [], 0
    for index, layout in enumerate(make_layouts(args.count)):
        box, bin_pose = layout['box'], layout['bin']
        label = (f'layout {index}: box ({box[0]:.3f}, {box[1]:.3f}, {math.degrees(box[2]):.1f}), '
                 f'bin ({bin_pose[0]:.3f}, {bin_pose[1]:.3f}, {math.degrees(bin_pose[2]):.1f})')
        if EXECUTOR_LOG.exists():
            EXECUTOR_LOG.unlink()
        rig = Rig(args.workspace, [
            'scene.enabled=true', 'enable_rgbd_camera=true', f'scene.box.x={box[0]}',
            f'scene.box.y={box[1]}', f'scene.box.yaw={box[2]}', f'scene.bin.x={bin_pose[0]}',
            f'scene.bin.y={bin_pose[1]}', f'scene.bin.yaw={bin_pose[2]}'], True,
            executor_params=['place.into_bin=true'], source=args.source)
        try:
            if not rig.spin_until(lambda: rig.start.get_subscription_count() > 0, 30):
                raise RuntimeError('executor did not come up')
            rig.start.publish(Empty())
            if not rig.spin_until(lambda: bool(rig.outcomes), 240):
                failures.append(f'{label}: no outcome within 240 s')
                continue
            rig.spin_until(lambda: False, 2.0)  # let the box settle in the recorded samples
            outcome = rig.outcomes[-1]
            generation = max(o.generation for o in rig.observations)
            mine = [o for o in rig.observations if o.generation == generation]
            targets = phase_targets()
            problems = []
            if args.source == 'vision':
                latched = [e[1] for e in rig.events if e[0] == 'latch' and
                           e[1].state == InitialPoseLatch.LATCHED]
                if not latched:
                    problems.append('never latched')
                    reference = None
                else:
                    pose = latched[-1].bin
                    reference = (pose.position.x, pose.position.y, pose.position.z)
            else:
                reference = (bin_pose[0], bin_pose[1], BIN_FLOOR_Z)
            ik_failed = outcome.failure_code == 'IK_FAILED'
            if reference is not None and not ik_failed:
                for phase in ('PREPLACE', 'PLACE'):
                    found = [t for t in targets if t[0] == phase]
                    if not found:
                        problems.append(f'no {phase} target in the log')
                        continue
                    _, x, y, z = found[-1]
                    dxy = 1000.0 * max(abs(x - reference[0]), abs(y - reference[1]))
                    if dxy > 1.0:
                        problems.append(f'{phase} target {dxy:.2f} mm from the bin')
                    if phase == 'PLACE' and abs(z - (reference[2] + 0.07)) > 0.001:
                        problems.append(f'PLACE z {z:.4f}, expected {reference[2] + 0.07:.4f}')
            inside = box_in_bin(mine[-1], bin_pose)
            geometry = release_geometry(mine)
            align_text = ''
            if args.align_check:
                problems = [p for p in problems if 'target' not in p or 'PLACE z' in p]
                if args.source == 'vision':
                    latched_box = [e[1].box for e in rig.events if e[0] == 'latch' and
                                   e[1].state == InitialPoseLatch.LATCHED]
                    box_yaw = math.degrees(latched_box[-1].yaw_rad) if latched_box else math.nan
                else:
                    box_yaw = math.degrees(yaw_of(mine[0].object_pose.pose.orientation))
                expected = fold(box_yaw, 90.0)
                rotations = logged_rotations()
                grasp = [r for p_, r in rotations if p_ == 'GRASP']
                placed = [r for p_, r in rotations if p_ == 'PLACE']
                runs, width = attach_runs_and_width(mine)
                if not grasp or abs(fold(grasp[0] - expected, 360.0)) > 1.0:
                    problems.append(f'C1/C2 GRASP rotation {grasp[:1]} vs box {expected:.2f}')
                if grasp and placed and abs(fold(placed[-1] - grasp[0], 360.0)) > 0.1:
                    problems.append(f'C1/C2 PLACE rotation {placed[-1]:.2f} != GRASP '
                                    f'{grasp[0]:.2f}')
                if runs != 1:
                    problems.append(f'C1/C2 {runs} attached runs')
                if width is None or width > 0.045:
                    problems.append(f'C1/C2 width at attach {width}')
                align_text = (f'; box yaw folded {expected:.2f}, GRASP rotation '
                              f'{grasp[0] if grasp else math.nan:.2f}, PLACE rotation '
                              f'{placed[-1] if placed else math.nan:.2f} deg; attached runs '
                              f'{runs}, width at attach '
                              f'{(width or math.nan) * 1000:.1f} mm')
            if args.source == 'oracle' and not ik_failed and not args.align_check:
                if not outcome.success or outcome.retries != 0:
                    problems.append(f'B1 outcome {outcome.failure_code}, retries '
                                    f'{outcome.retries}')
                else:
                    successes += 1
            if not ik_failed and not inside and not args.align_check:
                problems.append('the box is not inside the bin at the end')
            if geometry and geometry[0] < 0:
                problems.append(f'B3 fingertip {geometry[0]:.1f} mm below the wall top')
            place = [t for t in targets if t[0] == 'PLACE']
            print(f'{label}\n    outcome {outcome.failure_code} success={outcome.success} '
                  f'retries={outcome.retries}; box in bin: {inside}; PLACE target '
                  + (f'({place[-1][1]:.4f}, {place[-1][2]:.4f}, {place[-1][3]:.4f})' if place
                     else 'none')
                  + (f'; reference bin ({reference[0]:.4f}, {reference[1]:.4f}, '
                     f'{reference[2]:.4f})' if reference else '')
                  + (f'; at release fingertip {geometry[0]:.1f} mm above the wall top, box '
                     f'bottom {geometry[1]:.1f} mm above the floor; attachment runs '
                     f'{geometry[2]}' if geometry else '')
                  + align_text + f'  {"OK" if not problems else "FAIL"}', flush=True)
            failures += [f'{label}: {p}' for p in problems]
        finally:
            rig.close()
    if args.source == 'oracle' and successes == 0 and not args.align_check:
        failures.append('B1 no layout succeeded')
    print('\nPLACE ACCEPTANCE:', 'PASS' if not failures else 'FAIL')
    for failure in failures:
        print('  ', failure)
    return 0 if not failures else 1


def run_place_legacy(args):
    failures = []
    if EXECUTOR_LOG.exists():
        EXECUTOR_LOG.unlink()
    rig = Rig(args.workspace, ['enable_rgbd_camera=true'], True, source=args.source)
    try:
        if not rig.spin_until(lambda: rig.start.get_subscription_count() > 0, 30):
            raise RuntimeError('executor did not come up')
        for episode in range(args.episodes):
            rig.outcomes.clear()
            rig.start.publish(Empty())
            if not rig.spin_until(lambda: bool(rig.outcomes), 240):
                failures.append(f'episode {episode}: no outcome within 240 s')
                break
            outcome = rig.outcomes[-1]
            place = [t for t in phase_targets() if t[0] == 'PLACE']
            target = place[-1][1:] if place else None
            grasp = [r for p_, r in logged_rotations() if p_ == 'GRASP']
            if args.align_check and (not grasp or abs(grasp[-1]) > 1.0):
                failures.append(f'C4 episode {episode}: GRASP rotation {grasp[-1:]}')
            print(f'episode {episode}: success={outcome.success} retries={outcome.retries} '
                  f'PLACE target {target}, GRASP rotation '
                  f'{grasp[-1] if grasp else math.nan:.2f} deg', flush=True)
            if not outcome.success or outcome.retries != 0:
                failures.append(f'B4 episode {episode}: {outcome.failure_code}')
            if target is None or max(abs(a - b) for a, b in zip(target, (0.5, 0.3, 0.29))) > 1e-4:
                failures.append(f'B4 episode {episode}: PLACE target {target}')
    finally:
        rig.close()
    print('\nPLACE (legacy) ACCEPTANCE:', 'PASS' if not failures else 'FAIL')
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
    parser.add_argument('command', choices=['static', 'gaps', 'latch', 'place', 'episode'])
    parser.add_argument('--workspace', default=str(Path(__file__).resolve().parents[3]))
    parser.add_argument('--count', type=int, default=6)
    parser.add_argument('--align-check', action='store_true',
                        help='place: judge the Stage 9 tool alignment instead of B1/B2')
    parser.add_argument('--source', choices=['oracle', 'vision'], default='vision',
                        help='place: the executor\'s observation source')
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
    return {'static': run_static, 'gaps': run_gaps, 'latch': run_latch, 'place': run_place,
            'episode': run_episode}[args.command](args)


if __name__ == '__main__':
    sys.exit(main())
