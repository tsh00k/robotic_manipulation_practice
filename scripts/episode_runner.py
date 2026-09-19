#!/usr/bin/python3
"""Stage J episode runner (week2.md): drives task_executor through N consecutive
pick-and-place episodes via ~/start_episode, without restarting any process --
see task_executor_node.cpp's onStartEpisode() for the node-side half of this.
Each episode's ~/episode_outcome, plus the box's ground-truth pose observed at
that moment, gets appended as one row to a CSV under results/.

This is the deliberately-not-a-gtest layer 4 of week2.md 2.1.1's four-layer
scheme: task-level success rate over many minutes of physics, not a CI gate --
its output belongs in results/, not build/*/test_results/.

Must be run with /usr/bin/python3, not whatever `python3` resolves to on this
machine: pyenv's shim is bypassed by a pyenv-managed interpreter earlier on
PATH, which is missing rclpy's compiled extension. See CLAUDE.md, "变体二".

Usage:
    /usr/bin/python3 scripts/episode_runner.py --episodes 20 \
        --out results/pick_place_run.csv
"""
import argparse
import csv
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from manipulation_interfaces.msg import EpisodeOutcome
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future
from std_msgs.msg import Empty

# How long to wait for task_executor_node's ~/start_episode subscription to be
# discovered before giving up. DDS publishers do not queue messages for
# not-yet-discovered subscribers (no durability configured here), so publishing
# before discovery completes silently drops the message -- confirmed live: this
# runner's very first episode timed out with no corresponding log line at all on
# the task_executor_node side, until this wait was added.
DISCOVERY_TIMEOUT_S = 15.0

# Worst case an episode can take: max_retries+1 full attempts (fsm.hpp default
# max_retries=3, so 4) x up to phase_timeout_s (default 6.0s) x ~10 phases, plus
# reset/settle overhead. Generous on purpose -- a real timeout here should mean
# "the node stopped responding", not "the FSM was merely slow".
DEFAULT_TIMEOUT_S = 120.0

CSV_FIELDS = [
    'episode', 'seed', 'success', 'failure_code', 'retries', 'total_duration_s',
    'phase_names', 'phase_durations_s', 'final_box_x_m', 'final_box_y_m', 'final_box_z_m',
]


class EpisodeRunner(Node):

    def __init__(self, timeout_s):
        # use_sim_time: this node only measures wall-clock time to bound
        # rclpy.spin_until_future_complete's timeout (a runner-side safety net,
        # not a fact about the episode itself), so it deliberately does NOT set
        # use_sim_time -- unlike every node inside the sim graph. The per-episode
        # timing that matters (phase_durations_s) comes from task_executor's own
        # sim-time measurements inside EpisodeOutcome, not from this node's clock.
        super().__init__('episode_runner')
        self._timeout_s = timeout_s
        self._start_pub = self.create_publisher(Empty, '/task_executor/start_episode', 10)
        self._outcome_sub = self.create_subscription(
            EpisodeOutcome, '/task_executor/episode_outcome', self._on_outcome, 10)
        self._object_pose_sub = self.create_subscription(
            PoseStamped, '/mujoco_bridge/ground_truth/object_pose', self._on_object_pose, 10)
        self._latest_object_pose = None
        self._outcome_future = None

    def _on_object_pose(self, msg):
        self._latest_object_pose = msg

    def _on_outcome(self, msg):
        if self._outcome_future is not None and not self._outcome_future.done():
            self._outcome_future.set_result(msg)

    def _wait_for_start_episode_subscriber(self, timeout_s):
        # Cheap no-op once the first episode has already connected: subscription
        # count only ever goes to zero again if task_executor_node itself exits,
        # which is not a case this runner is designed to recover from anyway.
        deadline = self.get_clock().now() + Duration(seconds=timeout_s)
        while self._start_pub.get_subscription_count() == 0:
            if self.get_clock().now() > deadline:
                raise RuntimeError(
                    'no subscriber ever appeared on /task_executor/start_episode '
                    f'within {timeout_s}s -- is task_executor_node running?')
            rclpy.spin_once(self, timeout_sec=0.1)

    def run_episode(self):
        """Returns (outcome_or_None, final_pose_or_None). outcome is None on timeout."""
        self._wait_for_start_episode_subscriber(DISCOVERY_TIMEOUT_S)
        self._outcome_future = Future()
        self._start_pub.publish(Empty())
        rclpy.spin_until_future_complete(self, self._outcome_future, timeout_sec=self._timeout_s)
        if not self._outcome_future.done():
            return None, self._latest_object_pose
        return self._outcome_future.result(), self._latest_object_pose


def row_for(index, outcome, pose):
    if outcome is None:
        return {
            'episode': index,
            'seed': 0,
            'success': False,
            'failure_code': 'RUNNER_TIMEOUT',
            'retries': -1,
            'total_duration_s': '',
            'phase_names': '',
            'phase_durations_s': '',
            'final_box_x_m': pose.pose.position.x if pose else '',
            'final_box_y_m': pose.pose.position.y if pose else '',
            'final_box_z_m': pose.pose.position.z if pose else '',
        }
    return {
        'episode': index,
        # Fixed object pose this week (week2.md Stage I's KeyframeWaypointSource
        # ignores object_pose) -- a constant placeholder column, not a real seed,
        # until domain randomization lands (week2.md 6.1's hanging item on
        # WaypointSource's next replacement).
        'seed': 0,
        'success': outcome.success,
        'failure_code': outcome.failure_code,
        'retries': outcome.retries,
        'total_duration_s': sum(outcome.phase_durations_s),
        'phase_names': '|'.join(outcome.phase_names),
        'phase_durations_s': '|'.join(f'{d:.4f}' for d in outcome.phase_durations_s),
        'final_box_x_m': pose.pose.position.x if pose else '',
        'final_box_y_m': pose.pose.position.y if pose else '',
        'final_box_z_m': pose.pose.position.z if pose else '',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episodes', type=int, default=20)
    parser.add_argument('--out', type=Path, default=Path('results/pick_place_run.csv'))
    parser.add_argument('--timeout', type=float, default=DEFAULT_TIMEOUT_S)
    args = parser.parse_args()

    rclpy.init()
    runner = EpisodeRunner(args.timeout)
    rows = []
    try:
        for i in range(1, args.episodes + 1):
            runner.get_logger().info(f'episode {i}/{args.episodes}: starting')
            outcome, pose = runner.run_episode()
            row = row_for(i, outcome, pose)
            rows.append(row)
            if outcome is None:
                runner.get_logger().error(f'episode {i}: TIMED OUT after {args.timeout}s')
            else:
                runner.get_logger().info(
                    f"episode {i}: success={row['success']} failure_code={row['failure_code']} "
                    f"retries={row['retries']} total_duration_s={row['total_duration_s']:.2f}")
    finally:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with open(args.out, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            writer.writeheader()
            writer.writerows(rows)
        runner.get_logger().info(f'wrote {len(rows)} rows to {args.out}')
        runner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    n_success = sum(1 for r in rows if r['success'] is True)
    print(f'{n_success}/{len(rows)} episodes succeeded (see {args.out})')
    sys.exit(0 if n_success == len(rows) else 1)


if __name__ == '__main__':
    main()
