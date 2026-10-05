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
Capture one settled depth frame per random box/bin layout, with truth (Week 4.1 Stage 3).

    source install/setup.bash
    ROS_DOMAIN_ID=84 /usr/bin/python3 src/mujoco_perception/test/layout_capture.py \
        --seed 20261005 --count 40 --out /tmp/layouts_dev

Protocol (fixed before any detector ran, see week4.1 Stage 3):
  - box and bin centres uniform in x [0.38, 0.62], y [-0.22, 0.22]; roll = pitch = 0.
  - box yaw: 8 equal strata of [0, 90) deg, count/8 layouts each; bin yaw: 8 strata of
    [0, 180) deg; the pairing of strata is a random permutation.
  - a layout that the bridge itself refuses (box too close to the bin, off the table) is
    resampled from the same random stream and counted in layouts.json.
  - the arm is reset to the HOME keyframe through the bridge's reset service, then the
    scene settles for --settle-s of simulated time before the frame is taken.
Only the bridge runs (camera on). The truth (box pose from the bridge, bin pose from the
command line the bridge accepted) is for evaluation and is never given to a detector.
"""

import argparse
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
from manipulation_interfaces.msg import BridgeObservation
from manipulation_interfaces.srv import ResetScene
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=50)
X_RANGE = (0.38, 0.62)
Y_RANGE = (-0.22, 0.22)
TABLE_TOP = 0.22
# bin: floor 6 mm thick below the origin (inner floor surface), so the auto support height
# of an upright bin is table + 0.006 + 1 mm. Box: half size 0.02 plus 1 mm.
BIN_AUTO_Z = TABLE_TOP + 0.006 + 0.001
BOX_AUTO_Z = TABLE_TOP + 0.02 + 0.001


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def decode(msg, dtype, channels):
    row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    width_bytes = msg.width * channels * np.dtype(dtype).itemsize
    array = row[:, :width_bytes].copy().view(dtype)
    return array.reshape(msg.height, msg.width, channels).squeeze()


def make_layouts(seed, count):
    rng = np.random.default_rng(seed)
    per = count // 8
    box_stratum = np.repeat(np.arange(8), per)
    bin_stratum = rng.permutation(np.repeat(np.arange(8), per))
    layouts = []
    for i in range(count):
        layouts.append(dict(
            box_yaw=math.radians((box_stratum[i] + rng.random()) * 90.0 / 8),
            bin_yaw=math.radians((bin_stratum[i] + rng.random()) * 180.0 / 8)))
    return layouts, rng


def sample_position(rng):
    return dict(
        box_x=float(rng.uniform(*X_RANGE)), box_y=float(rng.uniform(*Y_RANGE)),
        bin_x=float(rng.uniform(*X_RANGE)), bin_y=float(rng.uniform(*Y_RANGE)))


class Capturer:
    def __init__(self, workspace, settle_s):
        self.executable = (Path(workspace) / 'install' / 'mujoco_bridge' / 'lib' /
                           'mujoco_bridge' / 'mujoco_bridge_node')
        self.settle_s = settle_s
        rclpy.init()
        self.node = rclpy.create_node('layout_capture')
        self.obs = []
        self.depth = {}
        self.rgb = {}
        self.node.create_subscription(
            BridgeObservation, '/mujoco_bridge/episode_observation', self.obs.append, 200)
        self.node.create_subscription(
            Image, '/mujoco_bridge/camera/depth/image_raw',
            lambda m: self.depth.__setitem__(stamp_ns(m.header.stamp), m), BEST_EFFORT)
        self.node.create_subscription(
            Image, '/mujoco_bridge/camera/color/image_raw',
            lambda m: self.rgb.__setitem__(stamp_ns(m.header.stamp), m), BEST_EFFORT)
        self.client = self.node.create_client(
            ResetScene, '/mujoco_bridge/reset_with_generation')

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return predicate()

    def one(self, layout, log_path):
        """Start a bridge for this layout; return ('ok', frame) or ('rejected', reason)."""
        self.obs.clear()
        self.depth.clear()
        self.rgb.clear()
        params = {
            'scene.enabled': 'true', 'enable_rgbd_camera': 'true',
            'scene.box.x': layout['box_x'], 'scene.box.y': layout['box_y'],
            'scene.box.yaw': layout['box_yaw'], 'scene.bin.x': layout['bin_x'],
            'scene.bin.y': layout['bin_y'], 'scene.bin.yaw': layout['bin_yaw']}
        args = [str(self.executable), '--ros-args']
        for key, value in params.items():
            args += ['-p', f'{key}:={value}']
        env = dict(os.environ, LIBGL_ALWAYS_SOFTWARE='1')
        log = open(log_path, 'w')
        process = subprocess.Popen(
            args, stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        try:
            up = self.spin_until(lambda: bool(self.obs) or process.poll() is not None, 45)
            if process.poll() is not None or not up:
                log.flush()
                text = Path(log_path).read_text()
                reason = next((line for line in text.splitlines() if 'startup failed' in line),
                              'bridge did not start')
                return 'rejected', reason
            if not self.client.wait_for_service(timeout_sec=10.0):
                return 'error', 'reset service missing'
            future = self.client.call_async(ResetScene.Request())
            self.spin_until(future.done, 10.0)
            generation = future.result().generation
            self.spin_until(lambda: any(o.generation == generation for o in self.obs), 10.0)
            first = next(o for o in self.obs if o.generation == generation)
            target = stamp_ns(first.joint_state.header.stamp) + int(self.settle_s * 1e9)
            self.spin_until(
                lambda: any(k >= target for k in self.depth), 90.0)
            keys = sorted(k for k in self.depth if k >= target)
            if not keys:
                return 'error', 'no depth frame after settling'
            key = keys[0]
            truth = next((o for o in self.obs if o.generation == generation and
                          stamp_ns(o.joint_state.header.stamp) == key), None)
            if truth is None:
                self.spin_until(lambda: any(
                    o.generation == generation and stamp_ns(o.joint_state.header.stamp) == key
                    for o in self.obs), 5.0)
                truth = next((o for o in self.obs if o.generation == generation and
                              stamp_ns(o.joint_state.header.stamp) == key), None)
            if truth is None:
                return 'error', 'no bridge sample at the depth stamp'
            self.spin_until(lambda: key in self.rgb, 1.0)
            p = truth.object_pose.pose
            frame = dict(
                depth=decode(self.depth[key], np.float32, 1),
                rgb=(decode(self.rgb[key], np.uint8, 3) if key in self.rgb
                     else np.zeros((1, 1, 3), np.uint8)),
                has_rgb=key in self.rgb, stamp_ns=key,
                box_pose=np.array([p.position.x, p.position.y, p.position.z,
                                   p.orientation.x, p.orientation.y, p.orientation.z,
                                   p.orientation.w]),
                width=np.array(sum(truth.joint_state.position[-2:])))
            return 'ok', frame
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            log.close()
            # Let DDS forget the old publisher before the next bridge starts.
            self.spin_until(lambda: self.node.count_publishers('/clock') == 0, 8.0)

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--count', type=int, default=40)
    parser.add_argument('--out', required=True)
    parser.add_argument('--workspace', default=str(Path(__file__).resolve().parents[3]))
    parser.add_argument('--settle-s', type=float, default=2.0)
    parser.add_argument('--only', type=int, nargs='*', default=None,
                        help='capture just these layout indices (debugging)')
    parser.add_argument('--record-seed-only', action='store_true',
                        help='write layouts.json with the seed and strata, capture nothing')
    args = parser.parse_args()
    if args.count % 8 != 0:
        parser.error('--count must be a multiple of 8 (8 yaw strata)')
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    layouts, rng = make_layouts(args.seed, args.count)
    meta = dict(seed=args.seed, count=args.count, rejected=[], layouts=[])
    if args.record_seed_only:
        (out / 'layouts.json').write_text(json.dumps(
            dict(seed=args.seed, count=args.count, captured=False), indent=1))
        print(f'recorded seed {args.seed} only, nothing captured')
        return 0

    capturer = Capturer(args.workspace, args.settle_s)
    if capturer.node.count_publishers('/clock') != 0:
        print('FAIL: a /clock publisher already exists; stop leftover nodes first')
        return 1
    try:
        for index, layout in enumerate(layouts):
            if args.only is not None and index not in args.only:
                layout.update(sample_position(rng))   # keep the random stream aligned
                meta['layouts'].append(dict(index=index, skipped=True))
                continue
            attempts = 0
            while True:
                layout.update(sample_position(rng))
                attempts += 1
                status, result = capturer.one(layout, f'/tmp/layout_capture_{index}.log')
                if status == 'rejected':
                    meta['rejected'].append(dict(index=index, layout=dict(layout), why=result))
                    print(f'layout {index}: rejected by the bridge ({result[-90:]}); resampling')
                    if attempts > 20:
                        raise RuntimeError('too many rejected layouts')
                    continue
                break
            if status != 'ok':
                print(f'layout {index}: FAILED ({result})')
                meta['layouts'].append(dict(index=index, error=result, **layout))
                continue
            frame = result
            np.savez_compressed(
                out / f'layout_{index:03d}.npz', depth=frame['depth'], rgb=frame['rgb'],
                has_rgb=frame['has_rgb'], stamp_ns=frame['stamp_ns'],
                box_pose=frame['box_pose'], width=frame['width'],
                bin_xyz_yaw=np.array([layout['bin_x'], layout['bin_y'], BIN_AUTO_Z,
                                      layout['bin_yaw']]))
            meta['layouts'].append(dict(index=index, attempts=attempts, **layout))
            print(f'layout {index}: box ({layout["box_x"]:.3f}, {layout["box_y"]:.3f}, '
                  f'{math.degrees(layout["box_yaw"]):5.1f} deg)  bin ({layout["bin_x"]:.3f}, '
                  f'{layout["bin_y"]:.3f}, {math.degrees(layout["bin_yaw"]):6.1f} deg)  '
                  f'attempts={attempts}', flush=True)
    finally:
        capturer.close()
        (out / 'layouts.json').write_text(json.dumps(meta, indent=1))
    print(f'done: {sum(1 for m in meta["layouts"] if "error" not in m and "skipped" not in m)} '
          f'captured, {len(meta["rejected"])} bridge rejections')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
