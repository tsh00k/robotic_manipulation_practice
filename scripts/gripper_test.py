#!/usr/bin/python3
"""Cycles the gripper open/closed via mujoco_bridge's ~/joint_command.

Manual verification tool, same spirit as sine_joint_test.py: run this against a
running mujoco_bridge_node and confirm in RViz (or `ros2 topic echo /joint_states`)
that left_finger/right_finger move together between open and closed. No assertion
here -- human-in-the-loop check, not a pytest-style test.

Commands both finger joints to the same target on purpose: the MJCF only exposes a
single tendon-driven actuator for the gripper (see week1.md 4.3.4 and 11.2), so
sending different targets for finger_joint1/finger_joint2 would just hit the
mujoco_bridge_node "commanded to different positions" warning and use whichever
arrived last -- there is no way to move the two fingers independently.

Must be run with /usr/bin/python3, not whatever `python3` resolves to on this
machine: pyenv's shim is bypassed by a pyenv-managed interpreter earlier on PATH,
which is missing rclpy's compiled extension. See CLAUDE.md, "变体二".
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Finger joint range per panda.xml's `finger` default class: <joint type="slide"
# range="0 0.04"/>. 0.0 = fully closed, 0.04 = fully open (4 cm of travel).
FINGER_JOINT_NAMES = ['finger_joint1', 'finger_joint2']
OPEN_POSITION_M = 0.04
CLOSED_POSITION_M = 0.0
HOLD_S = 2.0  # time to hold each state before switching -- long enough to watch
PUBLISH_RATE_HZ = 10.0


class GripperTest(Node):

    def __init__(self):
        # See sine_joint_test.py: without this, HOLD_S is measured against wall time
        # rather than /clock.
        super().__init__(
            'gripper_test',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.pub = self.create_publisher(JointTrajectory, '/mujoco_bridge/joint_command', 10)
        self.is_open = False
        self.start_time = self.get_clock().now()
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self.tick)
        self.get_logger().info(
            f'Cycling gripper between {CLOSED_POSITION_M:.3f} m (closed) and '
            f'{OPEN_POSITION_M:.3f} m (open), holding {HOLD_S}s each way')

    def tick(self):
        elapsed_s = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        want_open = int(elapsed_s // HOLD_S) % 2 == 1
        if want_open != self.is_open:
            self.is_open = want_open
            self.get_logger().info('target: %s' % ('open' if self.is_open else 'closed'))

        target = OPEN_POSITION_M if self.is_open else CLOSED_POSITION_M
        msg = JointTrajectory()
        msg.joint_names = FINGER_JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = [target, target]
        msg.points = [point]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = GripperTest()
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
