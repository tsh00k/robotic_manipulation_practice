#!/usr/bin/python3
"""Publishes a slow sine-wave JointTrajectory to mujoco_bridge's ~/joint_command.

Manual verification tool for Stage E: run this against a running mujoco_bridge_node
and confirm in RViz (or `ros2 topic echo /joint_states`) that joint4's TF frame
oscillates. There is no assertion here -- this is a human-in-the-loop check, not a
test in the pytest sense.

Must be run with /usr/bin/python3, not whatever `python3` resolves to on this
machine: pyenv's shim is bypassed by a pyenv-managed interpreter earlier on PATH,
which is missing rclpy's compiled extension. See CLAUDE.md, "变体二".
"""
import math

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# joint4's `home` keyframe angle (see panda.xml <key name="home">). Chosen as the
# oscillation center because its range (-3.0718, -0.0698) comfortably fits the
# amplitude below on both sides -- joint4 is the one arm joint whose range is not
# roughly symmetric around 0, so centering on 0 would risk clipping against the
# ctrlrange the MJCF enforces on this actuator.
JOINT_NAME = 'joint4'
CENTER_RAD = -1.57079
AMPLITUDE_RAD = 0.3
FREQUENCY_HZ = 0.1  # ~10s period -- "slow" per the Stage E plan, easy to eyeball
PUBLISH_RATE_HZ = 20.0


class SineJointTest(Node):

    def __init__(self):
        super().__init__('sine_joint_test')
        self.pub = self.create_publisher(JointTrajectory, '/mujoco_bridge/joint_command', 10)
        self.start_time = self.get_clock().now()
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self.tick)
        self.get_logger().info(
            f'Publishing sine target for {JOINT_NAME}: '
            f'{CENTER_RAD:.3f} +/- {AMPLITUDE_RAD:.3f} rad @ {FREQUENCY_HZ} Hz')

    def tick(self):
        elapsed_s = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        target = CENTER_RAD + AMPLITUDE_RAD * math.sin(2.0 * math.pi * FREQUENCY_HZ * elapsed_s)

        msg = JointTrajectory()
        msg.joint_names = [JOINT_NAME]
        point = JointTrajectoryPoint()
        point.positions = [target]
        msg.points = [point]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SineJointTest()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
