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
End-to-end checks of the bridge's optional box/bin start layout (scene.*).

Source the workspace under test first and use an otherwise unused ROS domain:
    source install/setup.bash
    ROS_DOMAIN_ID=84 /usr/bin/python3 src/mujoco_bridge/test/scene_probe.py <command>

Commands (each starts and cleans up its own bridge process, logs in /tmp):
  capture   Reset, wait for the scene to settle, save one snapshot (observation fields,
            optionally the RGB/depth frame) as JSON/NPZ. Run it once against the Stage 5
            baseline workspace and once against the workspace under test, then `compare`.
            With --expect-box / --expect-bin it also checks the pose right after each of
            two resets against what the command line asked for.
  compare   Compare two snapshots; exits non-zero if any field differs beyond tolerance.
  expect-fail  Start the bridge with invalid parameters and check that it exits with
            status 1 and a message naming the problem, not abort()/a core dump.

The bin has no topic or TF by design (the task must not read simulator ground truth), so
its pose can only be checked from the bridge's start-up log line.
"""

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

TABLE_TOP_Z = 0.22
BOX_HALF = np.array([0.02, 0.02, 0.02])
# Outer extent of the bin geoms in the bin frame (pick_place_scene.xml).
BIN_MIN = np.array([-0.076, -0.071, -0.006])
BIN_MAX = np.array([0.076, 0.071, 0.012])
POSITION_TOLERANCE_M = 0.002
ORIENTATION_TOLERANCE_RAD = 0.0175


def rotation(roll, pitch, yaw):
    """Rz(yaw) Ry(pitch) Rx(roll), written independently of the C++ implementation."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def auto_z(roll, pitch, yaw, lo, hi):
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1])
                        for z in (lo[2], hi[2])])
    return TABLE_TOP_Z - (corners @ rotation(roll, pitch, yaw).T)[:, 2].min() + 0.001


def quaternion_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def angle_between(a, b):
    cos = (np.trace(a.T @ b) - 1) / 2
    return float(np.arccos(np.clip(cos, -1.0, 1.0)))


def bridge_command(workspace, params):
    executable = Path(workspace) / 'install' / 'mujoco_bridge' / 'lib' / 'mujoco_bridge' / \
        'mujoco_bridge_node'
    command = [str(executable), '--ros-args']
    for item in params:
        command += ['-p', item.replace('=', ':=', 1)]
    return command


class Bridge:
    """A real bridge process, started directly (not through ros2 run) and always reaped."""

    def __init__(self, workspace, params, log_path, camera):
        env = dict(os.environ)
        if camera:
            env['LIBGL_ALWAYS_SOFTWARE'] = '1'
            params = list(params) + ['enable_rgbd_camera=true']
        self.log_path = log_path
        self.log = open(log_path, 'w')
        self.process = subprocess.Popen(
            bridge_command(workspace, params), stdout=self.log, stderr=subprocess.STDOUT,
            env=env, start_new_session=True)

    def stop(self):
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGTERM)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
        self.log.close()

    def log_text(self):
        self.log.flush()
        return Path(self.log_path).read_text()


def ros_imports():
    import rclpy
    from manipulation_interfaces.msg import BridgeObservation
    from manipulation_interfaces.srv import ResetScene
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    return rclpy, BridgeObservation, ResetScene, qos_profile_sensor_data, Image


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def decode_image(msg):
    channels = {'rgb8': 3, '32FC1': 1}[msg.encoding]
    dtype = np.uint8 if msg.encoding == 'rgb8' else np.float32
    itemsize = np.dtype(dtype).itemsize
    row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    array = row[:, :msg.width * channels * itemsize].copy().view(dtype)
    return array.reshape(msg.height, msg.width, channels).squeeze()


def snapshot_of(obs):
    pose = obs.object_pose.pose
    tcp = obs.world_to_hand_tcp.transform
    return {
        'generation': int(obs.generation),
        'stamp': stamp_s(obs.joint_state.header.stamp),
        'joint_names': list(obs.joint_state.name),
        'joint_position': list(obs.joint_state.position),
        'box_position': [pose.position.x, pose.position.y, pose.position.z],
        'box_orientation_xyzw': [pose.orientation.x, pose.orientation.y, pose.orientation.z,
                                 pose.orientation.w],
        'tcp_position': [tcp.translation.x, tcp.translation.y, tcp.translation.z],
        'tcp_orientation_xyzw': [tcp.rotation.x, tcp.rotation.y, tcp.rotation.z, tcp.rotation.w],
        'attachment_state': int(obs.attachment_state),
        'left_finger_contact': bool(obs.left_finger_contact),
        'right_finger_contact': bool(obs.right_finger_contact),
    }


def check_pose(label, snapshot, expected):
    """Check a box snapshot against (x, y, z|'auto', roll, pitch, yaw); return failures."""
    x, y, z, roll, pitch, yaw = expected
    if z == 'auto':
        z = auto_z(roll, pitch, yaw, -BOX_HALF, BOX_HALF)
    failures = []
    position = np.array(snapshot['box_position'])
    error = np.abs(position - np.array([x, y, z])).max()
    q = snapshot['box_orientation_xyzw']
    angle = angle_between(quaternion_to_matrix(*q), rotation(roll, pitch, yaw))
    print(f'  {label}: box position error {error * 1000:.3f} mm, orientation error '
          f'{np.degrees(angle):.3f} deg (expected z {z:.5f})')
    if error > POSITION_TOLERANCE_M:
        failures.append(f'{label}: box position off by {error * 1000:.2f} mm')
    if angle > ORIENTATION_TOLERANCE_RAD:
        failures.append(f'{label}: box orientation off by {np.degrees(angle):.2f} deg')
    return failures


def check_bin_log(log_text, expected):
    """Check the bin pose in the start-up log against (x, y, z|'auto', roll, pitch, yaw)."""
    x, y, z, roll, pitch, yaw = expected
    if z == 'auto':
        z = auto_z(roll, pitch, yaw, BIN_MIN, BIN_MAX)
    match = re.search(
        r'bin xyz=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\] rpy=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]',
        log_text)
    if not match:
        return ['no `bin xyz=[...] rpy=[...]` line in the bridge log (scene not enabled?)']
    logged = [float(v) for v in match.groups()]
    wanted = [x, y, z, roll, pitch, yaw]
    # The log prints positions to 0.1 mm and angles to 1 mrad.
    tolerance = [2e-4] * 3 + [1.5e-3] * 3
    print(f'  bin logged {logged}, expected {[round(v, 5) for v in wanted]}')
    return [f'bin value {i} logged {a} expected {b}' for i, (a, b, t) in
            enumerate(zip(logged, wanted, tolerance)) if abs(a - b) > t]


def parse_expected(values):
    return tuple(v if v == 'auto' else float(v) for v in values) if values else None


def run_capture(args):
    rclpy, BridgeObservation, ResetScene, sensor_qos, Image = ros_imports()
    rclpy.init()
    node = rclpy.create_node('scene_probe')
    observations = []
    images = {}
    node.create_subscription(
        BridgeObservation, '/mujoco_bridge/episode_observation', observations.append, 50)
    if args.camera:
        for name in ('color', 'depth'):
            node.create_subscription(
                Image, f'/mujoco_bridge/camera/{name}/image_raw',
                lambda msg, name=name: images.setdefault(name, []).append(msg), sensor_qos)
    client = node.create_client(ResetScene, '/mujoco_bridge/reset_with_generation')

    def spin(seconds_or_predicate, timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if callable(seconds_or_predicate) and seconds_or_predicate():
                return True
        return not callable(seconds_or_predicate)

    # Environment hygiene: a leftover bridge would also publish /clock and /observation.
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    if node.count_publishers('/clock') != 0:
        print('FAIL: a /clock publisher already exists; stop leftover nodes first')
        return 1

    failures = []
    bridge = Bridge(args.workspace, args.param, args.log, args.camera)
    try:
        if not spin(lambda: bool(observations) and node.count_publishers('/clock') == 1, 30.0):
            print('FAIL: bridge did not come up; see', args.log)
            print(bridge.log_text()[-1500:])
            return 1
        if not client.wait_for_service(timeout_sec=10.0):
            print('FAIL: reset service not available')
            return 1

        resets = []
        for index in range(2):
            observations.clear()
            future = client.call_async(ResetScene.Request())
            spin(future.done, 10.0)
            generation = future.result().generation
            spin(lambda: any(o.generation == generation for o in observations), 10.0)
            first = next(o for o in observations if o.generation == generation)
            resets.append((generation, snapshot_of(first)))
            target = stamp_s(first.joint_state.header.stamp) + args.settle_s
            spin(lambda: observations and stamp_s(observations[-1].joint_state.header.stamp)
                 >= target, 60.0)
            settled = next(o for o in observations if o.generation == generation and
                           stamp_s(o.joint_state.header.stamp) >= target)
            resets[-1] += (snapshot_of(settled),)
            if args.camera and index == 0:
                spin(lambda: all(
                    any(stamp_s(m.header.stamp) >= target for m in images.get(n, []))
                    for n in ('color', 'depth')), 30.0)
                frames = {n: next(m for m in images[n] if stamp_s(m.header.stamp) >= target)
                          for n in ('color', 'depth')}
                np.savez(Path(args.out).with_suffix('.npz'), rgb=decode_image(frames['color']),
                         depth=decode_image(frames['depth']))

        print(f'generations after reset: {[r[0] for r in resets]}')
        for index, (_, first, settled) in enumerate(resets):
            box = np.array(settled['box_position'])
            first_box = np.array(first['box_position'])
            print(f'  reset {index + 1}: first sample box {np.round(first_box, 5).tolist()}, '
                  f'after {args.settle_s} s {np.round(box, 5).tolist()}')
        if resets[1][0] <= resets[0][0]:
            failures.append('generation did not increase across resets')

        box_expected = parse_expected(args.expect_box)
        if box_expected:
            print('pose right after each reset vs the command line:')
            for index, (_, first, _) in enumerate(resets):
                failures += check_pose(f'reset {index + 1}', first, box_expected)
        bin_expected = parse_expected(args.expect_bin)
        if bin_expected:
            print('bin pose from the start-up log vs the command line:')
            failures += check_bin_log(bridge.log_text(), bin_expected)

        Path(args.out).write_text(json.dumps(resets[0][2], indent=1))
        print(f'snapshot written to {args.out}')
    finally:
        bridge.stop()
        node.destroy_node()
        rclpy.shutdown()
    for failure in failures:
        print('FAIL:', failure)
    print('PASS' if not failures else 'FAILED')
    return 1 if failures else 0


def run_compare(args):
    a = json.loads(Path(args.a).read_text())
    b = json.loads(Path(args.b).read_text())
    failures = []
    for key in sorted(set(a) | set(b)):
        if key in ('stamp', 'generation'):
            continue
        if key not in a or key not in b:
            failures.append(f'{key}: present in only one snapshot')
            continue
        if isinstance(a[key], list) and a[key] and isinstance(a[key][0], (int, float)):
            diff = float(np.abs(np.array(a[key]) - np.array(b[key])).max())
            print(f'  {key}: max abs difference {diff:.3e}')
            if diff > args.tolerance:
                failures.append(f'{key} differs by {diff:.3e}')
        else:
            same = a[key] == b[key]
            print(f'  {key}: {"identical" if same else "DIFFERENT"}')
            if not same:
                failures.append(f'{key} differs: {a[key]} vs {b[key]}')
    npz_a, npz_b = Path(args.a).with_suffix('.npz'), Path(args.b).with_suffix('.npz')
    if npz_a.exists() and npz_b.exists():
        fa, fb = np.load(npz_a), np.load(npz_b)
        rgb = int(np.abs(fa['rgb'].astype(int) - fb['rgb'].astype(int)).max())
        valid = np.isfinite(fa['depth']) & np.isfinite(fb['depth'])
        depth = float(np.abs(fa['depth'][valid] - fb['depth'][valid]).max())
        mismatch = int((np.isfinite(fa['depth']) != np.isfinite(fb['depth'])).sum())
        print(f'  rgb {fa["rgb"].shape}: max abs difference {rgb} levels')
        print(f'  depth {fa["depth"].shape}: max abs difference {depth:.3e} m, '
              f'finite-ness differs at {mismatch} pixels')
        if rgb > args.rgb_tolerance:
            failures.append(f'rgb differs by {rgb} levels')
        if depth > args.depth_tolerance or mismatch:
            failures.append(f'depth differs by {depth:.3e} m, {mismatch} pixels')
    elif npz_a.exists() != npz_b.exists():
        note = 'camera frames exist in only one snapshot: images not compared'
        if args.require_images:
            failures.append(note)
        else:
            print(f'  NOTE: {note} (pass --require-images to make this a failure)')
    for failure in failures:
        print('FAIL:', failure)
    print('IDENTICAL within tolerance' if not failures else 'DIFFERENT')
    return 1 if failures else 0


def run_expect_fail(args):
    bridge = Bridge(args.workspace, args.param, args.log, camera=False)
    try:
        try:
            code = bridge.process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            print('FAIL: the bridge kept running despite invalid parameters')
            return 1
        text = bridge.log_text()
    finally:
        bridge.stop()
    failures = []
    if code != 1:
        failures.append(f'exit status {code}, expected 1 (a negative value is a signal, '
                        f'-6 would be abort())')
    if 'startup failed:' not in text:
        failures.append('no `startup failed:` line in the log')
    if args.fragment not in text:
        failures.append(f'log does not mention `{args.fragment}`')
    reason = next((line for line in text.splitlines() if 'startup failed:' in line), '')
    print(f'exit status {code}; {reason.strip()}')
    for failure in failures:
        print('FAIL:', failure)
    print('PASS' if not failures else 'FAILED')
    return 1 if failures else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)

    def common(p):
        p.add_argument('--workspace', default=str(Path(__file__).resolve().parents[3]),
                       help='workspace whose install/ holds the bridge (default: this repo)')
        p.add_argument('--param', action='append', default=[], metavar='NAME=VALUE',
                       help='bridge ROS parameter, repeatable, e.g. scene.enabled=true')
        p.add_argument('--log', default='/tmp/scene_probe_bridge.log')

    capture = sub.add_parser('capture')
    common(capture)
    capture.add_argument('--out', required=True, help='snapshot JSON (frames go to .npz)')
    capture.add_argument('--camera', action='store_true', help='also capture an RGB-D frame')
    capture.add_argument('--settle-s', type=float, default=1.5, help='sim seconds after reset')
    capture.add_argument('--expect-box', nargs=6, metavar=('X', 'Y', 'Z', 'ROLL', 'PITCH', 'YAW'),
                         help="z may be 'auto'")
    capture.add_argument('--expect-bin', nargs=6, metavar=('X', 'Y', 'Z', 'ROLL', 'PITCH', 'YAW'))
    capture.set_defaults(run=run_capture)

    compare = sub.add_parser('compare')
    compare.add_argument('a')
    compare.add_argument('b')
    compare.add_argument('--tolerance', type=float, default=1e-6)
    compare.add_argument('--depth-tolerance', type=float, default=1e-4)
    compare.add_argument('--rgb-tolerance', type=int, default=1)
    compare.add_argument('--require-images', action='store_true')
    compare.set_defaults(run=run_compare)

    fail = sub.add_parser('expect-fail')
    common(fail)
    fail.add_argument('--fragment', required=True, help='text the error must contain')
    fail.set_defaults(run=run_expect_fail)

    args = parser.parse_args()
    sys.exit(args.run(args))


if __name__ == '__main__':
    main()
