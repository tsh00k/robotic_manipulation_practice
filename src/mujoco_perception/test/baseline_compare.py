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
Measure the EXISTING box detector on the Stage 3 layouts (Week 4.1 Stage 5 baseline).

    source /opt/ros/humble/setup.bash && source install/setup.bash
    colcon build --packages-select mujoco_perception      # builds the baseline_replay tool
    /usr/bin/python3 src/mujoco_perception/test/baseline_compare.py /tmp/layouts_dev

The layouts come from layout_capture.py (one settled depth frame per seeded box/bin layout,
plus the simulator truth). This script writes them to a flat binary, runs the C++ tool
baseline_replay (which calls the existing segmentDepth() and estimateBoxPose()), parses its
output, and then replays the two acceptance rules the live system applies after the geometry:

  tracker   a frame is MEASURED only if exactly one candidate cluster is valid and has
            confidence >= 0.20 (object_tracker.hpp, TrackerConfig::min_confidence);
  executor  the measured pose is used only if confidence >= 0.5, residual <= 5 mm and
            inlier ratio >= 0.7 (task_executor_config.hpp, vision_* defaults).

Truth is used in two places, both evaluation-only and never by the detector under test:
  - error numbers (position and yaw against the box pose in the layout file);
  - --keep-bin no (the default) erases the bin's pixels using the true bin pose. That is
    deliberately FAVOURABLE to the baseline: it only has to find the box. --keep-bin yes
    leaves the bin in the image, as the live system would see it.

What this does not measure: the robot mask, the tracker's temporal behaviour, the RGB/TF
pairing that makes the live node process only some frames. It compares the geometry only.
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
import initial_pose_eval as prototype  # noqa: E402  (camera model and the prototype detector)

REPO = Path(__file__).resolve().parents[3]

# Acceptance rules of the live system, copied from the sources named in the module docstring.
TRACKER_MIN_CONFIDENCE = 0.20
EXECUTOR_MIN_CONFIDENCE = 0.5
EXECUTOR_MAX_RESIDUAL_M = 0.005
EXECUTOR_MIN_INLIER_RATIO = 0.7
# Largest 3D OBB side estimateBoxPose() tolerates: 40 mm box + extent_tolerance_m (15 mm).
EXTENT_LIMIT_MM = 55.0
# Stage 2 result: the yaw error at which grasping stops being clean.
YAW_TOLERANCE_DEG = 30.0
# Bin outer half sizes (pick_place_bin_scene.xml) and how far past them pixels are erased.
BIN_HALF_X, BIN_HALF_Y, BIN_ERASE_MARGIN = 0.076, 0.071, 0.005
# Pixels of the bin are only erased below this height, so a box standing next to it is safe.
BIN_ERASE_MAX_Z = 0.245

CLUSTER_LINE = re.compile(
    r'\s+cluster (\d+) points=(\d+) obb_mm=(\S+) (\S+) (\S+) valid=(\d) reason=(\S+) '
    r'pos=(\S+) (\S+) (\S+) yaw_deg=(\S+) conf=(\S+) resid=(\S+) inl=(\S+)')


def load_layouts(directory):
    """Load every layout_*.npz of a dataset directory."""
    files = sorted(Path(directory).glob('layout_*.npz'))
    if not files:
        raise SystemExit(f'no layout_*.npz in {directory}')
    return [dict(np.load(f)) for f in files]


def erase_bin(depth, bin_xyz_yaw):
    """Set the depth pixels that belong to the bin to NaN, using the TRUE bin pose."""
    d = depth.astype(np.float64).copy()
    valid = np.isfinite(d) & (d > 0)
    world = prototype.backproject(np.where(valid, d, np.nan))
    bx, by, _, byaw = bin_xyz_yaw
    c, s = math.cos(byaw), math.sin(byaw)
    local_x = (world[..., 0] - bx) * c + (world[..., 1] - by) * s
    local_y = -(world[..., 0] - bx) * s + (world[..., 1] - by) * c
    inside = ((np.abs(local_x) < BIN_HALF_X + BIN_ERASE_MARGIN) &
              (np.abs(local_y) < BIN_HALF_Y + BIN_ERASE_MARGIN) &
              (world[..., 2] < BIN_ERASE_MAX_Z))
    d[inside] = np.nan
    return d.astype(np.float32)


def make_inputs(layouts, sigma_mm, frames, draws, keep_bin, rng):
    """
    Build the depth images to replay; return (images, index of the layout of each image).

    sigma_mm > 0 adds independent Gaussian noise to every valid pixel. frames > 1 averages
    that many independently noisy copies, which stands in for averaging consecutive frames of
    a static scene (the simulator's own depth has almost no frame-to-frame noise, so there is
    nothing real to average; this assumes the noise of different frames is independent).
    """
    images, owner = [], []
    for index, layout in enumerate(layouts):
        for _ in range(draws if sigma_mm > 0 else 1):
            d = layout['depth'].astype(np.float64).copy()
            valid = np.isfinite(d) & (d > 0)
            if sigma_mm > 0:
                noise = rng.normal(0.0, sigma_mm / 1000.0, (frames, int(valid.sum())))
                d[valid] += noise.mean(axis=0)
            d = d.astype(np.float32)
            if not keep_bin:
                d = erase_bin(d, layout['bin_xyz_yaw'])
            images.append(d)
            owner.append(index)
    return images, owner


def run_replay(harness, images):
    """Run the C++ tool on the images and return, per image, its list of cluster dicts."""
    path = Path('/tmp/baseline_compare_frames.bin')
    path.write_bytes(
        struct.pack('<i', len(images)) + b''.join(i.astype('<f4').tobytes() for i in images))
    result = subprocess.run([str(harness), str(path)], capture_output=True, text=True)
    if result.returncode != 0:
        raise SystemExit(f'{harness} failed:\n{result.stderr[-800:]}')
    per_image = []
    for line in result.stdout.splitlines():
        if line.startswith('frame '):
            per_image.append([])
            continue
        m = CLUSTER_LINE.match(line)
        if m:
            per_image[-1].append(dict(
                points=int(m.group(2)), obb_mm=[float(m.group(i)) for i in (3, 4, 5)],
                valid=m.group(6) == '1', reason=m.group(7), x=float(m.group(8)),
                y=float(m.group(9)), conf=float(m.group(12)), resid=float(m.group(13)),
                inlier=float(m.group(14))))
    if len(per_image) != len(images):
        raise SystemExit(f'expected {len(images)} frames from the tool, got {len(per_image)}')
    return per_image


def tracker_candidate(clusters):
    """Return the single eligible cluster the tracker would measure, else None."""
    eligible = [c for c in clusters if c['valid'] and c['points'] >= 3 and
                c['conf'] >= TRACKER_MIN_CONFIDENCE]
    return eligible[0] if len(eligible) == 1 else None


def executor_accepts(candidate):
    """Apply the executor's quality gate to a tracker-measured candidate."""
    return (candidate is not None and candidate['conf'] >= EXECUTOR_MIN_CONFIDENCE and
            candidate['resid'] <= EXECUTOR_MAX_RESIDUAL_M and
            candidate['inlier'] >= EXECUTOR_MIN_INLIER_RATIO)


def true_yaw_mod90(layout):
    """Return the box's true yaw folded to [-45, 45) degrees (a square repeats every 90)."""
    x, y, z, w = layout['box_pose'][3:7]
    yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return (yaw + 45.0) % 90.0 - 45.0


def report(label, layouts, per_image, owner):
    """Print how many frames survive each stage and how accurate the accepted ones are."""
    stage = collections.Counter()
    position_errors, yaw_errors, reasons = [], [], collections.Counter()
    accepted_by_yaw = collections.defaultdict(lambda: [0, 0])
    for clusters, index in zip(per_image, owner):
        layout = layouts[index]
        bucket = int(abs(true_yaw_mod90(layout)) // 15)
        accepted_by_yaw[bucket][0] += 1
        if not clusters:
            stage['no cluster'] += 1
            continue
        for c in clusters:
            if not c['valid']:
                reasons[c['reason']] += 1
        eligible = [c for c in clusters if c['valid'] and c['conf'] >= TRACKER_MIN_CONFIDENCE]
        if not eligible:
            stage['none eligible'] += 1
        elif len(eligible) > 1:
            stage['several eligible'] += 1
        elif executor_accepts(eligible[0]):
            stage['accepted'] += 1
            accepted_by_yaw[bucket][1] += 1
            box = layout['box_pose']
            position_errors.append(
                math.hypot(eligible[0]['x'] - box[0], eligible[0]['y'] - box[1]))
            # The existing estimator always reports yaw 0, so its yaw error is the true yaw.
            yaw_errors.append(abs(true_yaw_mod90(layout)))
        else:
            stage['rejected by the executor gate'] += 1
    total = len(per_image)
    print(f'{label}: {total} frames; ' + ', '.join(f'{k} {v}' for k, v in sorted(stage.items())) +
          f'  -> accepted {stage["accepted"]}/{total} ({100 * stage["accepted"] / total:.0f}%)')
    if position_errors:
        p = np.array(position_errors) * 1000.0
        y = np.array(yaw_errors)
        print(f'    accepted frames: position error median {np.median(p):.2f}, '
              f'p95 {np.percentile(p, 95):.2f}, max {p.max():.2f} mm; '
              f'yaw error (output is always 0) median {np.median(y):.1f}, max {y.max():.1f} deg')
    if reasons:
        print(f'    cluster rejections: {dict(reasons.most_common(4))}')
    print('    accepted / frames by true yaw (folded to 0..45 deg): ' + '  '.join(
        f'{15 * b}-{15 * b + 15}: {v[1]}/{v[0]}' for b, v in sorted(accepted_by_yaw.items())))


def explain_rejections(layouts, per_image, owner):
    """Print the OBB extents of every rejected cluster next to the box's true yaw."""
    print('rejected clusters (sigma 0): OBB extents are the three sorted sides, in mm')
    over = 0
    rejected = 0
    for clusters, index in zip(per_image, owner):
        for c in clusters:
            if c['valid']:
                continue
            rejected += 1
            over += c['obb_mm'][2] > EXTENT_LIMIT_MM
            print(f'  layout {index:2d}  true yaw {true_yaw_mod90(layouts[index]) % 90:5.1f} deg  '
                  f'points {c["points"]:3d}  OBB {c["obb_mm"][0]:5.1f} {c["obb_mm"][1]:5.1f} '
                  f'{c["obb_mm"][2]:5.1f}  {c["reason"]}')
    accepted = [c['obb_mm'][2] for clusters in per_image for c in clusters if c['valid']]
    print(f'  {over} of {rejected} rejected clusters have a side above {EXTENT_LIMIT_MM:.0f} mm')
    if accepted:
        print(f'  accepted clusters: largest side min {min(accepted):.1f}, '
              f'median {np.median(accepted):.1f}, max {max(accepted):.1f} mm '
              f'(limit {EXTENT_LIMIT_MM:.0f})')


def persistence(harness, layouts, sigma_mm, draws, keep_bin, rng):
    """Count, per layout, how many noisy draws the baseline accepts."""
    images, owner = make_inputs(layouts, sigma_mm, 1, draws, keep_bin, rng)
    per_image = run_replay(harness, images)
    accepted = collections.Counter()
    for clusters, index in zip(per_image, owner):
        accepted[index] += executor_accepts(tracker_candidate(clusters))
    histogram = collections.Counter(accepted[i] for i in range(len(layouts)))
    never, always = histogram.get(0, 0), histogram.get(draws, 0)
    print(f'sigma {sigma_mm:g} mm, {draws} draws per layout: layouts accepted k times: '
          f'{ {k: histogram.get(k, 0) for k in range(draws + 1)} }  '
          f'(never {never}, always {always}, sometimes {len(layouts) - never - always})')


def yaw_consequence(layouts, per_image):
    """Print how often the always-zero yaw exceeds the Stage 2 tolerance."""
    accepted = [abs(true_yaw_mod90(layouts[i])) for i, clusters in enumerate(per_image)
                if executor_accepts(tracker_candidate(clusters))]
    everything = [abs(true_yaw_mod90(layout)) for layout in layouts]
    a = np.array(accepted)
    print(f'accepted layouts: {len(a)}/{len(layouts)}; of those yaw error > '
          f'{YAW_TOLERANCE_DEG:.0f} deg: {(a > YAW_TOLERANCE_DEG).sum()}, > '
          f'{YAW_TOLERANCE_DEG / 2:.0f} deg: {(a > YAW_TOLERANCE_DEG / 2).sum()}; '
          f'all layouts > {YAW_TOLERANCE_DEG:.0f} deg: '
          f'{sum(e > YAW_TOLERANCE_DEG for e in everything)}')
    clean = sum(1 for e in a if e <= YAW_TOLERANCE_DEG)
    print(f'upper bound on layouts the baseline can pick up (accepted AND yaw within '
          f'{YAW_TOLERANCE_DEG:.0f} deg): {clean}/{len(layouts)}; derived from the Stage 2 '
          f'tolerance, not a run')


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('dataset', help='directory written by layout_capture.py')
    parser.add_argument('--harness', default=str(REPO / 'build/mujoco_perception/baseline_replay'))
    parser.add_argument('--keep-bin', choices=['yes', 'no'], default='no',
                        help='yes: leave the bin in the image; no: erase it using the true pose')
    parser.add_argument('--sigmas', type=float, nargs='*', default=[0.0, 1.0, 2.0],
                        help='depth noise in mm for the main table')
    parser.add_argument('--frames', type=int, nargs='*', default=[1],
                        help='frames averaged per detection (the table is printed per value)')
    parser.add_argument('--draws', type=int, default=3, help='noise draws per layout')
    parser.add_argument('--persistence-draws', type=int, default=5)
    parser.add_argument('--skip-extras', action='store_true',
                        help='skip the rejection explanation, persistence and yaw sections')
    args = parser.parse_args()

    layouts = load_layouts(args.dataset)
    keep_bin = args.keep_bin == 'yes'
    rng = np.random.default_rng(7)
    print(f'{len(layouts)} layouts; bin {"kept" if keep_bin else "erased with the true pose"}; '
          f'harness {args.harness}')
    for frames in args.frames:
        for sigma in args.sigmas:
            images, owner = make_inputs(layouts, sigma, frames, args.draws, keep_bin, rng)
            report(f'sigma {sigma:g} mm, N={frames}', layouts, run_replay(args.harness, images),
                   owner)
    if args.skip_extras:
        return
    images, owner = make_inputs(layouts, 0.0, 1, 1, keep_bin, rng)
    clean_run = run_replay(args.harness, images)
    print()
    explain_rejections(layouts, clean_run, owner)
    print()
    for sigma in (s for s in args.sigmas if s > 0):
        persistence(args.harness, layouts, sigma, args.persistence_draws, keep_bin, rng)
    print()
    yaw_consequence(layouts, clean_run)


if __name__ == '__main__':
    main()
