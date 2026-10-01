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
"""Evaluate a live robot mask against simulation truth outside the estimator."""

import argparse
import json
import time

import numpy as np
import rclpy
from manipulation_interfaces.msg import BridgeObservation, RobotMaskDiagnostics
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Empty
from tf2_msgs.msg import TFMessage


def stamp(message):
    return (message.header.stamp.sec, message.header.stamp.nanosec)


def rotation(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def translation(vector):
    return np.array([vector.x, vector.y, vector.z])


class Probe(Node):
    def __init__(self, start_episode):
        super().__init__('robot_mask_probe')
        self.depth = {}
        self.info = {}
        self.mask = {}
        self.foreground = {}
        self.diagnostics = {}
        self.observations = {}
        self.static_tf = {}
        self.start_publisher = self.create_publisher(
            Empty, '/task_executor/start_episode', 10) if start_episode else None
        self.create_subscription(
            Image, '/mujoco_bridge/camera/depth/image_raw',
            lambda msg: self.depth.__setitem__(stamp(msg), msg), qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, '/mujoco_bridge/camera/depth/camera_info',
            lambda msg: self.info.__setitem__(stamp(msg), msg), qos_profile_sensor_data)
        self.create_subscription(
            Image, '/object_pose_estimator/debug/robot_mask',
            lambda msg: self.mask.__setitem__(stamp(msg), msg), qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/object_pose_estimator/debug/foreground_points',
            lambda msg: self.foreground.__setitem__(stamp(msg), msg),
            qos_profile_sensor_data)
        self.create_subscription(
            RobotMaskDiagnostics,
            '/object_pose_estimator/debug/robot_mask_diagnostics',
            lambda msg: self.diagnostics.__setitem__(stamp(msg), msg), 10)
        self.create_subscription(
            BridgeObservation, '/mujoco_bridge/episode_observation',
            self.on_observation, 10)
        self.create_subscription(
            TFMessage, '/tf_static', self.on_static_tf,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_observation(self, message):
        header = message.joint_state.header
        self.observations[(header.stamp.sec, header.stamp.nanosec)] = message

    def on_static_tf(self, message):
        for transform in message.transforms:
            self.static_tf[transform.child_frame_id] = transform

    def report(self):
        camera = self.static_tf['camera_link'].transform
        optical = self.static_tf['camera_optical_frame'].transform
        world_rotation = rotation(camera.rotation) @ rotation(optical.rotation)
        world_translation = (
            rotation(camera.rotation) @ translation(optical.translation)
            + translation(camera.translation))
        matching = sorted(
            set(self.depth) & set(self.info) & set(self.mask)
            & set(self.foreground) & set(self.diagnostics) & set(self.observations))
        rows = []
        for key in matching:
            depth_msg, mask_msg = self.depth[key], self.mask[key]
            info, observation = self.info[key], self.observations[key]
            if self.diagnostics[key].status != 'NONE':
                continue
            depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(
                depth_msg.height, depth_msg.step // 4)[:, :depth_msg.width]
            mask = np.frombuffer(mask_msg.data, dtype=np.uint8).reshape(
                mask_msg.height, mask_msg.step)[:, :mask_msg.width]
            v, u = np.indices(depth.shape)
            optical_points = np.stack((
                (u - info.k[2]) * depth / info.k[0],
                (v - info.k[5]) * depth / info.k[4], depth), axis=-1)
            world_points = optical_points @ world_rotation.T + world_translation
            object_pose = observation.object_pose.pose
            object_rotation = rotation(object_pose.orientation)
            object_center = translation(object_pose.position)
            object_local = (world_points - object_center) @ object_rotation
            object_pixels = np.isfinite(depth) & np.all(
                np.abs(object_local) <= 0.024, axis=-1)
            cloud = point_cloud2.read_points(
                self.foreground[key], field_names=('x', 'y', 'z'), skip_nans=True)
            xyz = np.column_stack((cloud['x'], cloud['y'], cloud['z']))
            cloud_local = (xyz - object_center) @ object_rotation
            non_object_foreground = np.count_nonzero(
                np.any(np.abs(cloud_local) > 0.024, axis=-1))
            rows.append({
                'bridge_session': observation.bridge_session,
                'generation': observation.generation,
                'sample_sequence': observation.sample_sequence,
                'object_pixels': int(np.count_nonzero(object_pixels)),
                'object_pixels_masked': int(np.count_nonzero(object_pixels & (mask > 0))),
                'projected_pixels': self.diagnostics[key].projected_pixels,
                'masked_pixels': self.diagnostics[key].masked_pixels,
                'comparison_pixels': self.diagnostics[key].comparison_pixels,
                'mismatch_pixels': self.diagnostics[key].mismatch_pixels,
                'foreground_points': len(xyz),
                'foreground_outside_object': int(non_object_foreground),
                'mask_ms': self.diagnostics[key].elapsed_ms,
            })
        return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=12.0)
    parser.add_argument('--start-episode', action='store_true')
    parser.add_argument('--out', default='/tmp/r1_robot_mask_eval.json')
    args = parser.parse_args()
    rclpy.init()
    probe = Probe(args.start_episode)
    start = time.monotonic()
    started = False
    while time.monotonic() - start < args.seconds:
        rclpy.spin_once(probe, timeout_sec=0.1)
        if (probe.start_publisher is not None and not started
                and time.monotonic() - start >= 2.0
                and probe.start_publisher.get_subscription_count() > 0):
            probe.start_publisher.publish(Empty())
            started = True
    rows = probe.report()
    with open(args.out, 'w', encoding='utf-8') as output:
        json.dump(rows, output, indent=2)
    print('paired_frames', len(rows))
    print('object_pixels', sum(row['object_pixels'] for row in rows))
    print('object_pixels_masked', sum(row['object_pixels_masked'] for row in rows))
    print('foreground_outside_object', sum(
        row['foreground_outside_object'] for row in rows))
    print('report', args.out)
    probe.destroy_node()
    rclpy.shutdown()
    if not rows:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
