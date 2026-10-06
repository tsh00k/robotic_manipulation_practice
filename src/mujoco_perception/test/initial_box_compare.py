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
Compare the C++ initial box detector with the Python prototype on IDENTICAL inputs.

    source /opt/ros/humble/setup.bash && source install/setup.bash
    colcon build --packages-select mujoco_perception     # builds initial_box_replay
    /usr/bin/python3 src/mujoco_perception/test/initial_box_compare.py /tmp/layouts_dev

For every layout and noise draw it builds N noisy depth frames, then
  - feeds the N frames to the C++ tool initial_box_replay (DepthWindow mean, then
    detectInitialBox()), and
  - averages the same N frames in numpy and gives the mean to the prototype's detect()
    (initial_pose_eval.py, rectangle = OpenCV),
so any difference is the implementation, not the noise.

Acceptance, written before the first run:
  sigma = 0, one frame: on all layouts both detectors measure the box, and the C++ and the
    prototype differ by at most 0.1 mm in x, y and z and 0.1 degrees in yaw;
  sigma > 0: they agree on measured / not measured in at least 99 % of the detections, and
    the pairs where both measured differ by no more than the same amounts.
Truth is used only to report the C++ detector's own errors (the layout file's box pose); it is
never given to either detector. Bin pixels are left in the image, as the live system sees it.
"""

import argparse
import collections
import math
from pathlib import Path
import re
import struct
import subprocess
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import baseline_compare as common  # noqa: E402  (layout loading and the yaw fold)
import initial_pose_eval as prototype  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
POSITION_LIMIT_MM = 0.1
YAW_LIMIT_DEG = 0.1
AGREEMENT_LIMIT = 0.99
SPREAD_LIMIT_M = 0.02  # DepthWindow max_spread_m

DETECTION = re.compile(
    r'detection (\d+) measured=(\d) reason=(\S+) x=(\S+) y=(\S+) z=(\S+) yaw_deg=(\S+) '
    r'valid_px=(\d+)')
CANDIDATE = re.compile(
    r'\s+candidate \d+ pixels=(\d+) along_mm=(\S+) across_mm=(\S+) yaw_deg=(\S+) match=(\d)')


def noisy_frames(layout, sigma_mm, frames, rng):
    """Return `frames` copies of the layout's depth image with independent Gaussian noise."""
    base = layout['depth'].astype(np.float64)
    valid = np.isfinite(base) & (base > 0)
    out = []
    for _ in range(frames):
        d = base.copy()
        if sigma_mm > 0:
            d[valid] += rng.normal(0.0, sigma_mm / 1000.0, int(valid.sum()))
        out.append(d.astype(np.float32))
    return out


def window_mean(frames):
    """Per-pixel mean over the frames, the same rule as DepthWindow (valid fraction and spread)."""
    stack = np.stack([f.astype(np.float64) for f in frames])
    valid = np.isfinite(stack) & (stack > 0)
    total = np.where(valid, stack, 0.0).sum(axis=0)
    count = valid.sum(axis=0)
    mean = np.full(total.shape, np.nan)
    ok = count >= 0.5 * len(frames)
    # A pixel whose valid frames span more than 20 mm saw two surfaces: no measurement.
    smallest = np.where(valid, stack, np.inf).min(axis=0)
    largest = np.where(valid, stack, -np.inf).max(axis=0)
    ok &= (largest - smallest) <= SPREAD_LIMIT_M
    mean[ok] = total[ok] / count[ok]
    return mean.astype(np.float32)


def run_cpp(harness, detections, frames_per_detection):
    """Run initial_box_replay; detections is a list of lists of frames. Returns parsed rows."""
    path = Path('/tmp/initial_box_compare_frames.bin')
    with open(path, 'wb') as handle:
        handle.write(struct.pack('<ii', len(detections), frames_per_detection))
        for frames in detections:
            for frame in frames:
                handle.write(frame.astype('<f4').tobytes())
    result = subprocess.run([str(harness), str(path)], capture_output=True, text=True)
    if result.returncode != 0:
        raise SystemExit(f'{harness} failed:\n{result.stderr[-800:]}')
    rows = []
    for line in result.stdout.splitlines():
        m = DETECTION.match(line)
        if m:
            rows.append(dict(
                measured=m.group(2) == '1', reason=m.group(3), x=float(m.group(4)),
                y=float(m.group(5)), z=float(m.group(6)), yaw=float(m.group(7)),
                candidates=[]))
            continue
        m = CANDIDATE.match(line)
        if m:
            rows[-1]['candidates'].append(dict(
                pixels=int(m.group(1)), along=float(m.group(2)), across=float(m.group(3)),
                match=m.group(5) == '1'))
    if len(rows) != len(detections):
        raise SystemExit(f'expected {len(detections)} detections, got {len(rows)}')
    return rows


def run_prototype(depth):
    """Run the Python prototype on one depth image; return its box result or None."""
    box = prototype.detect(depth, sigma_m=0.0, frames=1, rect='opencv')['box']
    if box is None:
        return None
    return dict(x=float(box['xy'][0]), y=float(box['xy'][1]),
                z=float(box['top_z']) - 0.02, yaw=float(box['yaw90']))


def fold_difference(a, b, period=90.0):
    """Return a - b folded to [-period / 2, period / 2)."""
    return (a - b + period / 2.0) % period - period / 2.0


def compare(label, layouts, owner, cpp_rows, proto_rows):
    """Print agreement between C++ and prototype and the C++ detector's errors against truth."""
    n = len(cpp_rows)
    agree = sum(1 for c, p in zip(cpp_rows, proto_rows) if c['measured'] == (p is not None))
    both = [(c, p) for c, p in zip(cpp_rows, proto_rows) if c['measured'] and p is not None]
    dx = np.array([abs(c['x'] - p['x']) for c, p in both]) * 1000.0
    dy = np.array([abs(c['y'] - p['y']) for c, p in both]) * 1000.0
    dz = np.array([abs(c['z'] - p['z']) for c, p in both]) * 1000.0
    dyaw = np.array([abs(fold_difference(c['yaw'], p['yaw'])) for c, p in both])
    worst_pos = max(dx.max(), dy.max(), dz.max()) if len(both) else float('nan')
    worst_yaw = dyaw.max() if len(both) else float('nan')
    ok_pairs = len(both) == 0 or (worst_pos <= POSITION_LIMIT_MM and worst_yaw <= YAW_LIMIT_DEG)
    ok_agree = agree / n >= AGREEMENT_LIMIT
    print(f'{label}: {n} detections; C++ measured {sum(c["measured"] for c in cpp_rows)}, '
          f'prototype measured {sum(p is not None for p in proto_rows)}; '
          f'agree on measured/not {agree}/{n} ({100 * agree / n:.1f}%) '
          f'{"OK" if ok_agree else "FAIL"}')
    print(f'    pairs measured by both: {len(both)}; largest difference x,y,z '
          f'{worst_pos:.4f} mm, yaw {worst_yaw:.4f} deg  {"OK" if ok_pairs else "FAIL"}')
    errors, yaw_errors = [], []
    for c, index in zip(cpp_rows, owner):
        if c['measured']:
            box = layouts[index]['box_pose']
            errors.append(math.hypot(c['x'] - box[0], c['y'] - box[1]) * 1000.0)
            truth_yaw = common.true_yaw_mod90(layouts[index])
            yaw_errors.append(abs(fold_difference(c['yaw'], truth_yaw)))
    if errors:
        e, y = np.array(errors), np.array(yaw_errors)
        print(f'    C++ against truth ({len(e)} measured): position median {np.median(e):.2f}, '
              f'p95 {np.percentile(e, 95):.2f}, max {e.max():.2f} mm; yaw median '
              f'{np.median(y):.2f}, p95 {np.percentile(y, 95):.2f}, max {y.max():.2f} deg')
    misses = collections.Counter(c['reason'] for c in cpp_rows if not c['measured'])
    if misses:
        print(f'    C++ reasons when not measured: {dict(misses)}')
    return ok_agree and ok_pairs


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('dataset', help='directory written by layout_capture.py')
    parser.add_argument(
        '--harness', default=str(REPO / 'build/mujoco_perception/initial_box_replay'))
    parser.add_argument('--sigmas', type=float, nargs='*', default=[0.0, 1.0, 2.0, 4.0])
    parser.add_argument('--frames', type=int, nargs='*', default=[1, 10])
    parser.add_argument('--draws', type=int, default=3,
                        help='noise draws per layout when sigma > 0')
    args = parser.parse_args()

    layouts = common.load_layouts(args.dataset)
    rng = np.random.default_rng(20261006)
    all_ok = True
    print(f'{len(layouts)} layouts; harness {args.harness}')
    for frames in args.frames:
        for sigma in args.sigmas:
            groups, owner = [], []
            for index, layout in enumerate(layouts):
                for _ in range(args.draws if sigma > 0 else 1):
                    groups.append(noisy_frames(layout, sigma, frames, rng))
                    owner.append(index)
            cpp_rows = run_cpp(args.harness, groups, frames)
            proto_rows = [run_prototype(window_mean(g)) for g in groups]
            all_ok &= compare(f'sigma {sigma:g} mm, N={frames}', layouts, owner, cpp_rows,
                              proto_rows)
    print('\nACCEPTANCE (C++ agrees with the prototype within the limits written above): '
          + ('PASS' if all_ok else 'FAIL'))
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
