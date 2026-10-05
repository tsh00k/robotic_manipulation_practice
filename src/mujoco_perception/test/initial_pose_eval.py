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
Offline error table for initial box/bin pose detection (Week 4.1 Stage 3).

    /usr/bin/python3 src/mujoco_perception/test/initial_pose_eval.py /tmp/layouts_dev \
        --t-pos 20 --t-yaw 30 --t-place 30 --bin-yaw-sensitive no

The tolerances come from Stage 2 and are required arguments on purpose: the verdict is
computed against them and nothing here lets a threshold be tuned after the fact.

Detector v0 (depth only, no colour, no truth):
  1. back-project the depth image to world coordinates with the camera model of
     pick_place_scene.xml (position, orientation, fovy, image size);
  2. keep points over the table region;
  3. box: height band z in [0.245, 0.30] -> connected components in the image -> the
     minimum-area rectangle of each component's x-y points -> accept a component whose
     sides are both 40 +- 5 mm; exactly one must pass;
  4. bin: height band z in [0.2235, 0.243] -> same, sides 152 x 142 +- 8 mm.
Thresholds come from the known geometry (table 0.22, box 0.04 tall, bin walls 0.012 above
a floor 0.006 thick) plus noise margin, not from the data.
"""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np

H, W = 240, 320
FOVY = math.radians(50.0)
FOCAL = H / (2 * math.tan(FOVY / 2))
CX, CY = (W - 1) / 2, (H - 1) / 2
CAM_POS = np.array([0.5, -0.45, 1.0])
_AX = np.array([1.0, 0, 0])
_AY = np.array([0, 0.857, 0.514])
_AY = _AY / np.linalg.norm(_AY)
_AZ = np.cross(_AX, _AY)
R_WORLD_OPTICAL = np.stack([_AX, -_AY, -_AZ], axis=1)
U, V = np.meshgrid(np.arange(W), np.arange(H))

TABLE_Z = 0.22
ROI = (0.2, 0.8, -0.4, 0.4)
BOX_BAND = (0.245, 0.30)
BIN_BAND = (0.2235, 0.243)
BOX_SIDE, BOX_TOL = 0.040, 0.005
BIN_SIDES, BIN_TOL = (0.152, 0.142), 0.008
MIN_COMPONENT_PIXELS = 20
BOX_TOP_Z = 0.26
BIN_FLOOR_Z = 0.227


def backproject(depth):
    z = depth.astype(float)
    pts = np.stack([(U - CX) * z / FOCAL, (V - CY) * z / FOCAL, z], axis=-1)
    return pts @ R_WORLD_OPTICAL.T + CAM_POS


def project(point):
    p = (np.asarray(point) - CAM_POS) @ R_WORLD_OPTICAL
    return FOCAL * p[0] / p[2] + CX, FOCAL * p[1] / p[2] + CY, p[2]


def min_area_rect(xy, step=0.25):
    """Return (angle deg in [0,90), centre, sizes along angle and angle+90)."""
    best = None
    for theta in np.arange(0.0, 90.0, step):
        c, s = math.cos(math.radians(theta)), math.sin(math.radians(theta))
        r = xy @ np.array([[c, -s], [s, c]])
        lo, hi = r.min(0), r.max(0)
        area = float(np.prod(hi - lo))
        if best is None or area < best[0]:
            best = (area, theta, (lo + hi) / 2, hi - lo)
    _, theta, mid, size = best
    c, s = math.cos(math.radians(theta)), math.sin(math.radians(theta))
    return theta, np.array([[c, -s], [s, c]]) @ mid, size


def wrap(angle, period):
    return (angle + period / 2) % period - period / 2


def quat_yaw_deg(q):
    x, y, z, w = q
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def components(mask):
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), connectivity=8)
    return [labels == i for i in range(1, count)
            if stats[i, cv2.CC_STAT_AREA] >= MIN_COMPONENT_PIXELS]


def detect(depth, rng=None, sigma_m=0.0, frames=1):
    """
    Detect the box and the bin; each result is None or a dict.

    frames > 1 simulates averaging that many frames of a static scene, each with its own
    independent noise draw. That assumes the noise is independent between frames; a real
    sensor's fixed-pattern or bias component would not average out.
    """
    d = depth.astype(float).copy()
    ok = np.isfinite(d) & (d > 0)
    if sigma_m > 0:
        d[ok] += rng.normal(0.0, sigma_m, (frames, int(ok.sum()))).mean(axis=0)
    w = backproject(np.where(ok, d, np.nan))
    ok = ok & np.isfinite(w[..., 2])
    inside = ok & (w[..., 0] > ROI[0]) & (w[..., 0] < ROI[1]) & \
        (w[..., 1] > ROI[2]) & (w[..., 1] < ROI[3])

    def passing(band, sides, tol):
        """Components whose minimum-area rectangle has the two given side lengths."""
        band_mask = inside & (w[..., 2] > band[0]) & (w[..., 2] < band[1])
        want = sorted(sides)
        found = []
        for comp in components(band_mask):
            pts = w[comp]
            theta, centre, size = min_area_rect(pts[:, :2])
            if all(abs(a - b) < tol for a, b in zip(sorted(size), want)):
                found.append((comp, pts, theta, centre, size))
        return found

    box = passing(BOX_BAND, (BOX_SIDE, BOX_SIDE), BOX_TOL)
    bins = passing(BIN_BAND, BIN_SIDES, BIN_TOL)
    out = dict(box=None, bin=None, box_candidates=len(box), bin_candidates=len(bins))
    if len(box) == 1:
        _, pts, theta, centre, size = box[0]
        top = pts[pts[:, 2] > 0.255]
        out['box'] = dict(
            xy=centre, yaw90=wrap(theta, 90.0), size=size,
            top_z=float(np.median(top[:, 2])) if len(top) else float(np.median(pts[:, 2])))
    if len(bins) == 1:
        _, pts, theta, centre, size = bins[0]
        long_axis = theta if size[0] >= size[1] else theta + 90.0
        local = pts[:, :2] - centre
        c, s = math.cos(math.radians(long_axis)), math.sin(math.radians(long_axis))
        along = local @ np.array([c, s])
        across = local @ np.array([-s, c])
        floor = pts[(np.abs(along) < 0.056) & (np.abs(across) < 0.051)]
        out['bin'] = dict(
            xy=centre, dir180=long_axis % 180.0, size=size,
            floor_z=float(np.median(floor[:, 2])) if len(floor) else float('nan'))
    return out


def self_check(frames):
    """Camera model sanity on the first frames; raise if it does not hold."""
    lines = []
    for name, f in frames[:3]:
        depth = f['depth']
        w = backproject(depth)
        ok = np.isfinite(depth) & (depth > 0)
        z = w[..., 2][ok]
        # Only heights below the bin band: the bin floor (z = 0.227) is within 1 cm of the
        # table and would otherwise be counted as table.
        table = z[(z > TABLE_Z - 0.005) & (z < BIN_BAND[0])]
        box = f['box_pose']
        u, v, expected = project([box[0], box[1], box[2] + 0.02])
        measured = float(depth[int(round(v)), int(round(u))])
        shift = detect(depth)
        lines.append(
            f'  {name}: table-plane z std {table.std() * 1000:.2f} mm over {len(table)} px; '
            f'box top centre projects to ({u:.1f},{v:.1f}), depth measured {measured:.4f} '
            f'expected {expected:.4f} (diff {abs(measured - expected) * 1000:.1f} mm)')
        if table.std() > 0.001 or abs(measured - expected) > 0.004:
            raise RuntimeError('camera model self-check failed:\n' + '\n'.join(lines))
        if shift['box'] is not None:
            wrong = np.linalg.norm(shift['box']['xy'] - (box[:2] + np.array([0.03, 0.0]))) * 1000
            lines.append(f'    wrong-truth check: truth shifted by 30 mm gives an apparent '
                         f'error of {wrong:.1f} mm (must be near 30)')
    return lines


def percentile_row(values):
    if len(values) == 0:
        return 'n=0'
    a = np.abs(np.asarray(values, float))
    return (f'n={len(a):3d} median {np.median(a):6.2f}  p95 {np.percentile(a, 95):6.2f}  '
            f'max {a.max():6.2f}')


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('dataset')
    parser.add_argument('--t-pos', type=float, required=True,
                        help='Stage 2 grasp position tolerance, mm')
    parser.add_argument('--t-yaw', type=float, required=True, help='Stage 2 box yaw tol, deg')
    parser.add_argument('--t-place', type=float, required=True, help='Stage 2 place tol, mm')
    parser.add_argument('--bin-yaw-sensitive', choices=['yes', 'no'], required=True)
    parser.add_argument('--t-bin-yaw', type=float, default=None,
                        help='deg, required if --bin-yaw-sensitive yes')
    parser.add_argument('--sigmas', type=float, nargs='*', default=[0.0, 1.0, 2.0, 4.0],
                        help='depth noise in mm')
    parser.add_argument('--trials', type=int, default=3)
    parser.add_argument('--frames', type=int, default=1,
                        help='frames averaged per detection (1 = detector v0)')
    parser.add_argument('--json', default=None)
    args = parser.parse_args()
    if args.bin_yaw_sensitive == 'yes' and args.t_bin_yaw is None:
        parser.error('--t-bin-yaw is required when --bin-yaw-sensitive is yes')

    files = sorted(Path(args.dataset).glob('layout_*.npz'))
    frames = [(p.name, dict(np.load(p))) for p in files]
    print(f'{len(frames)} layouts from {args.dataset}; '
          f'frames averaged per detection: {args.frames}')
    print('self-check:')
    print('\n'.join(self_check(frames)))

    rng = np.random.default_rng(1)
    results = {}
    for sigma in args.sigmas:
        rows = []
        for name, f in frames:
            trials = 1 if sigma == 0 else args.trials
            for t in range(trials):
                det = detect(f['depth'], rng, sigma / 1000.0, args.frames)
                bp, binp = f['box_pose'], f['bin_xyz_yaw']
                row = dict(layout=name, trial=t, box_found=det['box'] is not None,
                           bin_found=det['bin'] is not None,
                           box_cands=det['box_candidates'], bin_cands=det['bin_candidates'],
                           box_truth_yaw=quat_yaw_deg(bp[3:]), bin_truth_yaw=math.degrees(binp[3]),
                           box_truth_xy=bp[:2].tolist(), bin_truth_xy=binp[:2].tolist())
                if det['box']:
                    b = det['box']
                    row.update(box_pos_mm=float(np.linalg.norm(b['xy'] - bp[:2]) * 1000),
                               box_yaw_deg=float(wrap(b['yaw90'] - row['box_truth_yaw'], 90.0)),
                               box_top_mm=float((b['top_z'] - (bp[2] + 0.02)) * 1000))
                if det['bin']:
                    b = det['bin']
                    row.update(bin_pos_mm=float(np.linalg.norm(b['xy'] - binp[:2]) * 1000),
                               bin_yaw180_deg=float(
                                   wrap(b['dir180'] - row['bin_truth_yaw'], 180.0)),
                               bin_yaw90_deg=float(wrap(b['dir180'] - row['bin_truth_yaw'], 90.0)),
                               bin_floor_mm=float((b['floor_z'] - binp[2]) * 1000))
                rows.append(row)
        results[sigma] = rows

    print('\n=== error table (all trials pooled per sigma) ===')
    verdicts = []
    for sigma, rows in results.items():
        n = len(rows)
        bf = [r for r in rows if r['box_found']]
        nf = [r for r in rows if r['bin_found']]
        print(f'\n-- depth noise sigma = {sigma:g} mm, {n} detections attempted')
        print(f'   box found {len(bf)}/{n} ({100 * len(bf) / n:.0f}%); '
              f'bin found {len(nf)}/{n} ({100 * len(nf) / n:.0f}%)')
        print('   box position mm   ', percentile_row([r['box_pos_mm'] for r in bf]))
        print('   box yaw deg (mod 90)', percentile_row([r['box_yaw_deg'] for r in bf]))
        print('   box top height mm ', percentile_row([r['box_top_mm'] for r in bf]))
        print('   bin position mm   ', percentile_row([r['bin_pos_mm'] for r in nf]))
        print('   bin yaw deg (mod 180)', percentile_row([r['bin_yaw180_deg'] for r in nf]))
        print('   bin yaw deg (mod 90) ', percentile_row([r['bin_yaw90_deg'] for r in nf]))
        print('   bin floor height mm', percentile_row([r['bin_floor_mm'] for r in nf]))

        def rule(values, tol, label):
            if not values:
                return False, f'{label}: no detections'
            a = np.abs(values)
            ok = np.percentile(a, 95) <= tol / 4 and a.max() <= tol / 2
            return ok, (f'{label}: p95 {np.percentile(a, 95):.2f} <= {tol / 4:.2f} and '
                        f'max {a.max():.2f} <= {tol / 2:.2f} -> {"OK" if ok else "FAIL"}')
        checks = [
            rule([r['box_pos_mm'] for r in bf], args.t_pos, 'box position'),
            rule([r['box_yaw_deg'] for r in bf], args.t_yaw, 'box yaw'),
            rule([r['bin_pos_mm'] for r in nf], args.t_place, 'bin position')]
        if args.bin_yaw_sensitive == 'yes':
            checks.append(rule([r['bin_yaw180_deg'] for r in nf], args.t_bin_yaw, 'bin yaw'))
        miss_ok = len(bf) == n and len(nf) == n
        print('   acceptance (p95 <= T/4, max <= T/2, no misses):')
        for _, text in checks:
            print('     ', text)
        print(f'      misses: box {n - len(bf)}, bin {n - len(nf)} -> '
              f'{"OK" if miss_ok else "FAIL"}')
        verdicts.append((sigma, all(ok for ok, _ in checks) and miss_ok))

    print('\n=== verdict per noise level (rule fixed before measurement) ===')
    for sigma, ok in verdicts:
        gate = 'sigma <= 2 mm must pass' if sigma <= 2 else 'reported only'
        print(f'  sigma {sigma:g} mm: {"PASS" if ok else "FAIL"}  ({gate})')

    print('\n=== breakdowns at sigma = 0 ===')
    base = results[min(results)]
    for lo in range(0, 90, 15):
        rows = [r for r in base if r['box_found'] and
                lo <= (r['box_truth_yaw'] % 90) < lo + 15]
        errors = percentile_row([r['box_yaw_deg'] for r in rows])
        print(f'   box yaw {lo:2d}-{lo + 15:2d} deg: yaw err {errors}')
    for name, pred in (('box y < 0', lambda r: r['box_truth_xy'][1] < 0),
                       ('box y >= 0', lambda r: r['box_truth_xy'][1] >= 0)):
        rows = [r for r in base if r['box_found'] and pred(r)]
        print(f'   {name:11s}: pos err {percentile_row([r["box_pos_mm"] for r in rows])}')
    for name, pred in (('bin y < 0.1', lambda r: r['bin_truth_xy'][1] < 0.1),
                       ('bin y >= 0.1', lambda r: r['bin_truth_xy'][1] >= 0.1)):
        rows = [r for r in base if r['bin_found'] and pred(r)]
        print(f'   {name:12s}: pos err {percentile_row([r["bin_pos_mm"] for r in rows])}')
    if args.json:
        Path(args.json).write_text(json.dumps(
            {str(k): v for k, v in results.items()}, default=float, indent=1))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
