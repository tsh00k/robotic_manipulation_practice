#!/usr/bin/python3
# Copyright 2026 anby
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Compare PCL registration on captured, offline truth-labelled object pixels."""

import argparse
import gzip
import json
from pathlib import Path
import subprocess

from manipulation_interfaces.msg import BridgeObservation
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage

from robot_mask_probe import rotation, translation


def read(path, kind):
    with gzip.open(path, 'rb') as source:
        return deserialize_message(source.read(), kind)


def evaluate(destination, name, points, center, anchored):
    pcd = destination / (name + '.pcd')
    with pcd.open('w', encoding='ascii') as output:
        output.write('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n')
        output.write(f'WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n')
        np.savetxt(output, points, fmt='%.8f')
    command = ['build/mujoco_perception/compare_registration', str(pcd),
               *map(str, center.tolist()), '1' if anchored else '0']
    result = subprocess.run(command, capture_output=True, text=True, check=True, timeout=15)
    rows = []
    for line in result.stdout.splitlines():
        algorithm, converged, error, elapsed, planar = line.split(',')
        rows.append({'frame': name, 'points': len(points), 'algorithm': algorithm,
                     'accepted_or_converged': bool(int(converged)),
                     'error_m': float(error), 'elapsed_ms': float(elapsed),
                     'planar': bool(int(planar))})
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    args = parser.parse_args()
    static = read(args.recording / 'static_tf.cdr.gz', TFMessage)
    transforms = {t.child_frame_id: t.transform for t in static.transforms}
    camera, optical = transforms['camera_link'], transforms['camera_optical_frame']
    world_rotation = rotation(camera.rotation) @ rotation(optical.rotation)
    world_translation = (translation(camera.translation)
                         + rotation(camera.rotation) @ translation(optical.translation))
    frames = sorted((p for p in args.recording.iterdir() if p.is_dir()),
                    key=lambda p: tuple(map(int, p.name.split('_'))))
    results = []
    destination = args.recording.parent / 'registration'
    destination.mkdir(exist_ok=True)
    for frame in frames:
        depth = read(frame / 'depth.cdr.gz', Image)
        info = read(frame / 'depth_info.cdr.gz', CameraInfo)
        observation = read(frame / 'observation.cdr.gz', BridgeObservation)
        z = np.frombuffer(depth.data, dtype=np.float32).reshape(
            depth.height, depth.step // 4)[:, :depth.width]
        v, u = np.indices(z.shape)
        xyz = np.stack(((u - info.k[2]) * z / info.k[0],
                        (v - info.k[5]) * z / info.k[4], z), axis=-1)
        xyz = xyz @ world_rotation.T + world_translation
        truth = observation.object_pose.pose
        center = translation(truth.position)
        local = (xyz - center) @ rotation(truth.orientation)
        object_pixels = np.isfinite(z) & np.all(np.abs(local) <= 0.021, axis=-1)
        points = xyz[object_pixels]
        if len(points) < 20:
            continue
        results.extend(evaluate(destination, frame.name, points, center, center[2] < 0.252))
    fixtures = [('supported_top', 0.24, False), ('supported_partial', 0.24, True),
                ('elevated_top', 0.34, False)]
    for name, center_z, partial in fixtures:
        points = np.array([[0.5 + x, y, center_z + 0.02]
                           for x in np.linspace(-0.01 if partial else -0.02, 0.02, 9)
                           for y in np.linspace(-0.02, 0.02, 9)])
        results.extend(evaluate(destination, name, points,
                                np.array([0.5, 0.0, center_z]), center_z < 0.252))
    summary = {}
    for algorithm in ['OBB', 'ICP', 'GICP']:
        rows = [r for r in results if r['algorithm'] == algorithm]
        accepted = [r for r in rows if r['accepted_or_converged']]
        summary[algorithm] = {
            'frames': len(rows),
            'accepted_or_converged': sum(r['accepted_or_converged'] for r in rows),
            'planar_frames': sum(r['planar'] for r in rows),
            'error_m_p50_p95': np.percentile([r['error_m'] for r in rows], [50, 95]).tolist(),
            'accepted_error_m_p50_p95': np.percentile(
                [r['error_m'] for r in accepted], [50, 95]).tolist() if accepted else [],
            'elapsed_ms_p50_p95': np.percentile(
                [r['elapsed_ms'] for r in rows], [50, 95]).tolist(),
        }
    (destination / 'frames.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
    (destination / 'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print(json.dumps(summary, indent=2))
    assert results, 'No visible object fixture'


if __name__ == '__main__':
    main()
