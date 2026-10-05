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
Measure how much pose error a pick-and-place episode tolerates (Week 4.1 Stage 2).

Source the workspace under test and use an otherwise unused ROS domain:
    source install/setup.bash
    ROS_DOMAIN_ID=84 /usr/bin/python3 src/task_executor/test/tolerance_probe.py \
        grasp-offset --values 0 5 10 15 20 25 30

Experiments (each starts and reaps its own bridge and executor):
  grasp-offset  vision-source executor fed a synthetic MEASURED pose = truth + offset, the
                offset applied only until the first ATTACHED sample of an episode.
  box-yaw       scene.box.yaw sweep, oracle source (tool yaw fixed, so the yaw is the
                relative error between box and gripper).
  place-offset  executor placement target (and its verification target) shifted, bin fixed.
  bin-yaw       scene.bin.yaw sweep, placement target at the bin centre.
  control       offset 0 three times and a 60 mm offset once, to check the injection works.

A run is clean when the executor reports success with zero retries AND the box finally lies
inside the bin (judged from the bridge's true box pose, evaluation only: no node under test
ever sees it). Truth is used for reporting; it never feeds a decision.
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from manipulation_interfaces.msg import BridgeObservation, EpisodeOutcome, VisionObjectPose
from std_msgs.msg import Empty

# Inner faces of the bin walls in the bin frame (pick_place_bin_scene.xml: walls 6 mm thick
# centred at +-0.073 / +-0.068 from the origin), and the box half size.
BIN_INNER_X = 0.070
BIN_INNER_Y = 0.065
BOX_HALF = 0.02
# Box centre height above which the box rests on a wall rim instead of the floor.
ON_FLOOR_Z_MAX = 0.252
DEFAULT_BIN = (0.5, 0.3, 0.0)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def box_in_bin(box_xy, box_yaw, box_z, bin_pose):
    """Return True if all four box corners lie inside the bin's inner walls."""
    bx, by, byaw = bin_pose
    c, s = math.cos(byaw), math.sin(byaw)
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx = box_xy[0] + BOX_HALF * (sx * math.cos(box_yaw) - sy * math.sin(box_yaw))
            cy = box_xy[1] + BOX_HALF * (sx * math.sin(box_yaw) + sy * math.cos(box_yaw))
            lx = c * (cx - bx) + s * (cy - by)
            ly = -s * (cx - bx) + c * (cy - by)
            if abs(lx) > BIN_INNER_X or abs(ly) > BIN_INNER_Y:
                return False
    return box_z < ON_FLOOR_Z_MAX


class Session:
    """A bridge plus an executor, started directly and always reaped."""

    def __init__(self, workspace, bridge_params, executor_params, source, tag):
        self.install = Path(workspace) / 'install'
        self.processes = []
        self.logs = []
        self.source = source
        self.offset = (0.0, 0.0)
        self.generation = -1
        self.attached_seen = False
        self.samples = []
        self.outcomes = []
        self.injected = 0

        rclpy.init()
        self.node = rclpy.create_node('tolerance_probe')
        self.start = self.node.create_publisher(Empty, '/task_executor/start_episode', 10)
        self.vision = self.node.create_publisher(
            VisionObjectPose, '/object_pose_estimator/object_pose', 10)
        self.node.create_subscription(
            BridgeObservation, '/mujoco_bridge/episode_observation', self.on_bridge, 200)
        self.node.create_subscription(
            EpisodeOutcome, '/task_executor/episode_outcome', self.outcomes.append, 10)
        for _ in range(20):
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if self.node.count_publishers('/clock') != 0:
            raise RuntimeError('a /clock publisher already exists; stop leftover nodes first')

        nodes = [
            ('mujoco_bridge', 'mujoco_bridge_node',
             ['--ros-args'] + self.pairs(bridge_params)),
            ('task_executor', 'task_executor_node',
             ['--ros-args', '-p', 'use_sim_time:=true', '-p', f'observation_source:={source}'] +
             self.pairs(executor_params)),
        ]
        for package, name, arguments in nodes:
            log = open(f'/tmp/tolerance_probe_{package}.log', 'w')
            self.logs.append(log)
            self.processes.append(subprocess.Popen(
                [str(self.install / package / 'lib' / package / name)] + arguments,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        ready = self.spin_until(
            lambda: bool(self.samples) and self.start.get_subscription_count() > 0, 60)
        if not ready or self.node.count_publishers('/clock') != 1:
            self.close()
            raise RuntimeError(f'nodes did not come up ({tag}); see /tmp/tolerance_probe_*.log')

    @staticmethod
    def pairs(items):
        out = []
        for item in items:
            out += ['-p', item.replace('=', ':=', 1)]
        return out

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return predicate()

    def on_bridge(self, msg):
        pose = msg.object_pose.pose
        attached = msg.attachment_state == BridgeObservation.ATTACHMENT_ATTACHED
        if msg.generation != self.generation:
            self.generation = msg.generation
            self.attached_seen = False
        self.attached_seen = self.attached_seen or attached
        positions = dict(zip(msg.joint_state.name, msg.joint_state.position))
        self.samples.append((
            msg.generation, msg.attachment_state, pose.position.x, pose.position.y,
            pose.position.z, yaw_of(pose.orientation),
            positions.get('finger_joint1', 0.0) + positions.get('finger_joint2', 0.0)))
        if self.source != 'vision':
            return
        dx, dy = (0.0, 0.0) if self.attached_seen else self.offset
        out = VisionObjectPose()
        out.header = msg.object_pose.header
        out.header.frame_id = 'world'
        out.bridge_session = msg.bridge_session
        out.generation = msg.generation
        out.sample_sequence = msg.sample_sequence
        out.evidence_state = VisionObjectPose.MEASURED
        out.pose = pose
        out.pose.position.x += dx
        out.pose.position.y += dy
        out.confidence = 1.0
        out.residual_m = 0.0
        out.inlier_ratio = 1.0
        self.vision.publish(out)
        self.injected += 1

    def run_episode(self, timeout_s=180.0):
        self.outcomes.clear()
        self.samples.clear()
        self.start.publish(Empty())
        if not self.spin_until(lambda: bool(self.outcomes), timeout_s):
            return None
        outcome = self.outcomes[-1]
        self.spin_until(lambda: False, 1.5)  # let the box settle in the recorded samples
        generation = max(s[0] for s in self.samples)
        mine = [s for s in self.samples if s[0] == generation]
        first_attached = next((s for s in mine if s[1] == BridgeObservation.ATTACHMENT_ATTACHED),
                              None)
        initial = mine[0]
        return dict(
            outcome=outcome, final=mine[-1], initial=initial, first_attached=first_attached,
            max_width=max(s[6] for s in mine))

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
        self.node.destroy_node()
        rclpy.shutdown()


def summarise(label, run, bin_pose):
    if run is None:
        return dict(label=label, clean=False, note='TIMEOUT')
    outcome, final, initial, attached = (
        run['outcome'], run['final'], run['initial'], run['first_attached'])
    inside = box_in_bin((final[2], final[3]), final[5], final[4], bin_pose)
    shift = (math.hypot(attached[2] - initial[2], attached[3] - initial[3]) * 1000
             if attached else float('nan'))
    clean = bool(outcome.success) and outcome.retries == 0 and inside
    return dict(
        label=label, clean=clean, success=bool(outcome.success), retries=int(outcome.retries),
        failure=outcome.failure_code, inside_bin=inside, attach_shift_mm=round(shift, 1),
        final_off_bin_centre_mm=round(
            math.hypot(final[2] - bin_pose[0], final[3] - bin_pose[1]) * 1000, 1),
        final_z=round(final[4], 4))


def print_row(row):
    shown = {k: v for k, v in row.items() if k != 'label'}
    print(f"{row['label']:>22s}  " + '  '.join(f'{k}={v}' for k, v in shown.items()), flush=True)


def run_conditions(args, conditions):
    """Run each condition in its own bridge/executor session; episodes within share it."""
    rows = []
    for cond in conditions:
        session = Session(
            args.workspace, cond['bridge'], cond['executor'], cond['source'], cond['label'])
        try:
            for repeat in range(cond.get('repeat', 1)):
                for item in cond['episodes']:
                    session.offset = item.get('offset', (0.0, 0.0))
                    run = session.run_episode()
                    suffix = f' #{repeat + 1}' if cond.get('repeat', 1) > 1 else ''
                    label = item['label'] + suffix
                    row = summarise(label, run, cond['bin'])
                    row['group'] = cond['group']
                    rows.append(row)
                    print_row(row)
        finally:
            session.close()
    return rows


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('experiment', choices=[
        'grasp-offset', 'box-yaw', 'place-offset', 'bin-yaw', 'control'])
    parser.add_argument('--values', nargs='*', type=float, default=None,
                        help='mm for offsets, degrees for yaw')
    parser.add_argument('--repeat', type=int, default=1)
    parser.add_argument('--diagonal', action='store_true',
                        help='grasp-offset: the four diagonal directions, --values is the '
                             'total magnitude in mm')
    parser.add_argument('--axes', nargs='+', choices=['x', 'y'], default=['x', 'y'],
                        help='grasp-offset: which world axes to offset')
    parser.add_argument('--workspace', default=str(Path(__file__).resolve().parents[3]))
    parser.add_argument('--out', default=None, help='append JSON lines here')
    args = parser.parse_args()

    scene = ['scene.enabled=true']
    conditions = []
    if args.experiment in ('grasp-offset', 'control'):
        episodes = []
        if args.experiment == 'control':
            episodes = [dict(label='offset 0 mm', offset=(0.0, 0.0))] * 3 + [
                dict(label='x +60 mm', offset=(0.06, 0.0))]
        elif args.diagonal:
            for value in args.values:
                m = value / 1000.0 / math.sqrt(2.0)
                for sx in (1, -1):
                    for sy in (1, -1):
                        episodes.append(dict(
                            label=f'diag {sx:+d}x {sy:+d}y {value:.0f} mm',
                            offset=(sx * m, sy * m)))
        else:
            for value in args.values:
                m = value / 1000.0
                if 'x' in args.axes:
                    episodes.append(dict(label=f'x {value:+.0f} mm', offset=(m, 0.0)))
                if 'y' in args.axes and (value != 0 or 'x' not in args.axes):
                    episodes.append(dict(label=f'y {value:+.0f} mm', offset=(0.0, m)))
        conditions.append(dict(
            group=args.experiment, label='grasp', source='vision', bridge=scene, executor=[],
            bin=DEFAULT_BIN, episodes=episodes, repeat=args.repeat))
    elif args.experiment == 'box-yaw':
        for deg in args.values:
            conditions.append(dict(
                group='box-yaw', label=f'yaw {deg}', source='oracle',
                bridge=scene + [f'scene.box.yaw={math.radians(deg):.6f}'], executor=[],
                bin=DEFAULT_BIN, episodes=[dict(label=f'box yaw {deg:+.0f} deg')],
                repeat=args.repeat))
    elif args.experiment == 'place-offset':
        for axis in ('x', 'y'):
            for mm in args.values:
                if axis == 'y' and mm == 0:
                    continue
                m = mm / 1000.0
                tx = DEFAULT_BIN[0] + (m if axis == 'x' else 0.0)
                ty = DEFAULT_BIN[1] + (m if axis == 'y' else 0.0)
                conditions.append(dict(
                    group='place-offset', label=f'{axis} {mm}', source='oracle', bridge=scene,
                    executor=[f'target.place_x_m={tx:.4f}', f'target.place_y_m={ty:.4f}',
                              f'verify.place_x_m={tx:.4f}', f'verify.place_y_m={ty:.4f}'],
                    bin=DEFAULT_BIN, episodes=[dict(label=f'place {axis} {mm:+.0f} mm')],
                    repeat=args.repeat))
    elif args.experiment == 'bin-yaw':
        for deg in args.values:
            rad = math.radians(deg)
            conditions.append(dict(
                group='bin-yaw', label=f'bin yaw {deg}', source='oracle',
                bridge=scene + [f'scene.bin.yaw={rad:.6f}'], executor=[],
                bin=(DEFAULT_BIN[0], DEFAULT_BIN[1], rad),
                episodes=[dict(label=f'bin yaw {deg:+.0f} deg')], repeat=args.repeat))

    rows = run_conditions(args, conditions)
    if args.out:
        with open(args.out, 'a') as handle:
            for row in rows:
                handle.write(json.dumps(row) + '\n')
    bad = [r for r in rows if not r['clean']]
    print(f'SUMMARY {args.experiment}: {len(rows) - len(bad)}/{len(rows)} clean')
    return 0


if __name__ == '__main__':
    sys.exit(main())
