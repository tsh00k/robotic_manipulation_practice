#!/usr/bin/env python3
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
"""Live RGB-D geometry smoke test; run with /usr/bin/python3 after enabling the camera."""

import math
import struct
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from manipulation_interfaces.msg import BridgeObservation
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage


def rotate(q, p):
    x, y, z, w = q
    vx, vy, vz = p
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


class Probe(Node):
    def __init__(self):
        super().__init__('stage_p_camera_probe')
        self.rgb = {}
        self.depth = {}
        self.info = {}
        self.depth_info = {}
        self.observations = {}
        self.tf = {}
        self.oracles = {}
        self.result = None
        self.create_subscription(Image, '/mujoco_bridge/camera/color/image_raw',
                                 self.on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, '/mujoco_bridge/camera/depth/image_raw',
                                 self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/mujoco_bridge/camera/color/camera_info',
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/mujoco_bridge/camera/depth/camera_info',
                                 self.on_depth_info, qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf_static', self.on_tf,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PoseStamped, '/mujoco_bridge/ground_truth/object_pose',
                                 self.on_oracle, 10)
        self.create_subscription(BridgeObservation, '/mujoco_bridge/episode_observation',
                                 self.on_observation, 10)

    def on_rgb(self, msg):
        self.rgb[(msg.header.stamp.sec, msg.header.stamp.nanosec)] = msg
        self.rgb = dict(list(self.rgb.items())[-5:])
        self.check()

    def on_info(self, msg):
        self.info[(msg.header.stamp.sec, msg.header.stamp.nanosec)] = msg
        self.info = dict(list(self.info.items())[-5:])
        self.check()

    def on_depth_info(self, msg):
        self.depth_info[(msg.header.stamp.sec, msg.header.stamp.nanosec)] = msg
        self.depth_info = dict(list(self.depth_info.items())[-5:])
        self.check()

    def on_tf(self, msg):
        for transform in msg.transforms:
            self.tf[transform.child_frame_id] = transform
        self.check()

    def on_oracle(self, msg):
        self.oracles[(msg.header.stamp.sec, msg.header.stamp.nanosec)] = msg
        self.oracles = dict(list(self.oracles.items())[-30:])
        self.check()

    def on_observation(self, msg):
        stamp = msg.joint_state.header.stamp
        self.observations[(stamp.sec, stamp.nanosec)] = msg
        self.observations = dict(list(self.observations.items())[-30:])
        self.check()

    def on_depth(self, msg):
        self.depth[(msg.header.stamp.sec, msg.header.stamp.nanosec)] = msg
        self.depth = dict(list(self.depth.items())[-5:])
        self.check()

    def check(self):
        if self.result is not None:
            return
        if 'camera_link' not in self.tf or 'camera_optical_frame' not in self.tf:
            return
        matches = (self.depth.keys() & self.rgb.keys() & self.info.keys() &
                   self.depth_info.keys() & self.observations.keys() & self.oracles.keys())
        if not matches:
            return
        stamp = max(matches)
        msg = self.depth[stamp]
        rgb = self.rgb[stamp]
        info = self.info[stamp]
        assert tuple(info.k) == tuple(self.depth_info[stamp].k)
        observation = self.observations[stamp]
        assert observation.bridge_session != 0
        assert observation.sample_sequence > 0
        assert msg.encoding == '32FC1' and rgb.encoding == 'rgb8'
        assert (msg.header.frame_id == rgb.header.frame_id == info.header.frame_id ==
                'camera_optical_frame')
        assert (msg.width, msg.height, msg.step) == (320, 240, 1280)
        assert (rgb.width, rgb.height, rgb.step) == (320, 240, 960)

        def world_point(u, v):
            depth = struct.unpack_from('<f', msg.data, v * msg.step + u * 4)[0]
            assert math.isfinite(depth) and 0.1 < depth < 2.0, depth
            point = ((u - info.k[2]) * depth / info.k[0],
                     (v - info.k[5]) * depth / info.k[4], depth)
            for frame in ('camera_optical_frame', 'camera_link'):
                transform = self.tf[frame].transform
                q = transform.rotation
                point = rotate((q.x, q.y, q.z, q.w), point)
                point = (point[0] + transform.translation.x,
                         point[1] + transform.translation.y,
                         point[2] + transform.translation.z)
            return depth, point

        depth, point = world_point(160, 120)
        _, table_point = world_point(160, 180)
        box = self.oracles[stamp].pose.position
        # Centre ray is aimed at the box; it must intersect its upper surface.
        error_xy = math.hypot(point[0] - box.x, point[1] - box.y)
        error_top = abs(point[2] - (box.z + 0.02))
        rgb_values = bytes(rgb.data)
        unique_colors = len({rgb_values[i:i + 3] for i in range(0, len(rgb_values), 300)})
        assert unique_colors > 10, unique_colors
        assert error_xy < 0.035 and error_top < 0.025, (point, box)
        assert abs(table_point[2] - 0.22) < 0.02, table_point
        self.result = (stamp, observation.generation, observation.sample_sequence,
                       depth, point, (box.x, box.y, box.z),
                       error_xy, error_top, table_point, unique_colors)


def main():
    rclpy.init()
    node = Probe()
    deadline = time.monotonic() + 15.0
    try:
        while node.result is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.result is None:
            raise RuntimeError('timed out: RGB={}, depth={}, info={}, depth_info={}, '
                               'observation={}, TF={}, oracle={}'.format(
                                   len(node.rgb), len(node.depth), len(node.info),
                                   len(node.depth_info), len(node.observations),
                                   list(node.tf), len(node.oracles)))
        print('PASS stamp={} generation={} sequence={} center_depth_m={:.4f} '
              'box_top_point={} oracle={} '
              'xy_error_m={:.4f} top_error_m={:.4f} table_point={} sampled_colors={}'.format(
                  *node.result))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(f'FAIL: {exc}', file=sys.stderr)
        raise
