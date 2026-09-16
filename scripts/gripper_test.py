#!/usr/bin/python3
"""Cycles the gripper open/closed via mujoco_bridge's ~/gripper_command.

Manual verification tool, same spirit as sine_joint_test.py: run this against a
running mujoco_bridge_node and confirm in RViz (or `ros2 topic echo /joint_states`)
that left_finger/right_finger move together between open and closed. No assertion
here -- human-in-the-loop check, not a pytest-style test.

Stage H (week2.md) split the gripper out of ~/joint_command into its own topic using
control_msgs/GripperCommand, matching the shape a real ros2_control gripper action
server expects. `position` there is the *total* finger-to-finger opening in meters
(not each finger's own 0..0.04 travel), matching the real Franka gripper's own
convention -- see mujoco_bridge_node.cpp's onGripperCommand for why.

Must be run with /usr/bin/python3, not whatever `python3` resolves to on this
machine: pyenv's shim is bypassed by a pyenv-managed interpreter earlier on PATH,
which is missing rclpy's compiled extension. See CLAUDE.md, "变体二".
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from control_msgs.msg import GripperCommand

# Total finger-to-finger opening. Each finger's own travel is panda.xml's `finger`
# default class (<joint type="slide" range="0 0.04"/>), so the total ranges 0 (fully
# closed) to 0.08 (fully open, 2 x 0.04).
OPEN_WIDTH_M = 0.08
CLOSED_WIDTH_M = 0.0
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
        self.pub = self.create_publisher(GripperCommand, '/mujoco_bridge/gripper_command', 10)
        self.is_open = False
        self.start_time = self.get_clock().now()
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self.tick)
        self.get_logger().info(
            f'Cycling gripper between {CLOSED_WIDTH_M:.3f} m (closed) and '
            f'{OPEN_WIDTH_M:.3f} m (open) total width, holding {HOLD_S}s each way')

    def tick(self):
        elapsed_s = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        want_open = int(elapsed_s // HOLD_S) % 2 == 1
        if want_open != self.is_open:
            self.is_open = want_open
            self.get_logger().info('target: %s' % ('open' if self.is_open else 'closed'))

        msg = GripperCommand()
        msg.position = OPEN_WIDTH_M if self.is_open else CLOSED_WIDTH_M
        msg.max_effort = 0.0
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
