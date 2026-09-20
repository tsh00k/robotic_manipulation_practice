#!/usr/bin/env bash
# Stage J (week2.md 11.8): records a demo rosbag while episode_runner.py drives
# task_executor through N pick-and-place episodes.
#
# Assumes the demo (mujoco_bridge_node + task_executor_node) is ALREADY RUNNING,
# started separately -- this script does not launch it. Deliberately: wrapping
# `ros2 launch` (itself a Python wrapper forking multiple node processes) inside
# a script that also needs to shut down `ros2 bag record` cleanly is exactly the
# class of process-lifecycle problem STUDY_NOTES_GUIDE.md 3.1 already spent two
# sessions diagnosing for `ros2 run` -- keeping this script's scope to "start the
# recorder, run N episodes via the existing episode_runner.py, stop the
# recorder" means it only has to solve ONE new lifecycle problem, not re-solve
# an already-painful one.
#
# Usage:
#   ros2 launch mujoco_bridge demo.launch.py   # in one terminal, left running
#   scripts/record_demo_bag.sh [--episodes N] [--timeout S]   # in another
set -euo pipefail

EPISODES=1
TIMEOUT_S=120
while [ $# -gt 0 ]; do
  case "$1" in
    --episodes) EPISODES="$2"; shift 2 ;;
    --timeout) TIMEOUT_S="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 1 ;;
  esac
done

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

# Authoritative environment-hygiene check (STUDY_NOTES_GUIDE.md 3.1): unlike
# that section's usual "confirm nothing is running before starting", this
# script assumes the demo is already up, so the check here is the opposite --
# confirm exactly ONE instance of each node, not zero (demo not started) and
# not more than one (duplicate publishers would silently corrupt the recording,
# per STUDY_NOTES_GUIDE.md 3.2's "measure the wrong thing" failure class).
# `-o comm` matches by executable name, not `-o args`, which self-matches the
# bash -c command line running this check (the pkill -f pitfall's grep cousin).
bridge_count=$(ps -eo pid,comm | awk '$2 ~ /^mujoco_bridge_n/ {c++} END{print c+0}')
executor_count=$(ps -eo pid,comm | awk '$2 ~ /^task_executor_n/ {c++} END{print c+0}')
if [ "$bridge_count" -ne 1 ] || [ "$executor_count" -ne 1 ]; then
  echo "refusing to record: expected exactly 1 mujoco_bridge_node and 1" >&2
  echo "task_executor_node, found $bridge_count and $executor_count. Start the" >&2
  echo "demo first: ros2 launch mujoco_bridge demo.launch.py" >&2
  exit 1
fi
clock_publishers=$(ros2 topic info /clock 2>/dev/null | awk -F': ' '/Publisher count/ {print $2}')
if [ "${clock_publishers:-0}" -ne 1 ]; then
  echo "refusing to record: /clock Publisher count is '${clock_publishers:-<none>}', expected 1" >&2
  exit 1
fi

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
out_dir="results/bags/pick_place_${timestamp}"
mkdir -p results/bags

echo "recording to ${out_dir}"
# --use-sim-time: stamps bag metadata off /clock (which mujoco_bridge_node
# already publishes) instead of the recorder process's own wall clock, so bag
# duration matches sim time, not however long this script happened to take.
# -a (all topics), not a curated whitelist: this is a one-off demo capture, not
# episode_runner.py's curated experiment CSV -- no topic list to keep in sync
# as topics get added or renamed.
ros2 bag record -a --use-sim-time -o "$out_dir" &
recorder_pid=$!

# Give the recorder a moment to actually subscribe before publishing starts
# flowing -- /tf_static is transient_local (Stage C) specifically so a late
# subscriber still gets it, but most topics here are volatile and DDS does not
# queue for a not-yet-discovered subscriber (the same class of race
# episode_runner.py's DISCOVERY_TIMEOUT_S already guards against on a different
# pub/sub edge).
sleep 3

# Known limitation, not fixed here: with -a, the recorder discovers each topic
# dynamically as it appears rather than subscribing to a fixed list up front.
# episode_runner.py's own ~/start_episode publisher does not exist until the
# command below starts, so episode 1's trigger message can still race the
# recorder's discovery of that specific topic and go unrecorded even though
# the sleep above already passed -- observed live: run 2026-09-20 recorded
# /task_executor/start_episode Count: 2 for a 3-episode run. Its *effects*
# (the resulting /task_executor/episode_outcome, the object's trajectory in
# /mujoco_bridge/ground_truth/object_pose) are still captured for every
# episode; only the trigger message itself is at risk, and only for episode 1.
# See week2.md 11.8 -- this is the same "assume the other side hasn't
# discovered you yet" DDS race this project has hit three times already
# (11.4), not a new failure mode.
/usr/bin/python3 scripts/episode_runner.py \
  --episodes "$EPISODES" \
  --out "${out_dir}/episodes.csv" \
  --timeout "$TIMEOUT_S"
episode_runner_status=$?

echo "stopping recorder (pid ${recorder_pid})"
kill -INT "$recorder_pid" 2>/dev/null || true
wait "$recorder_pid" 2>/dev/null || true

# Belt and suspenders: `ros2 bag record` was found (see week2.md 11.8) to run
# the recorder in-process rather than forking a child binary and, unlike
# `ros2 run`/`ros2 launch` (see the same section -- a bare kill -INT to those
# is a no-op, not a clean stop), to actually honor a bare kill -INT sent
# outside a terminal. Still confirm rather than assume, the same discipline
# STUDY_NOTES_GUIDE.md 3.1 applies to every other background process in this
# project. Checked by PID, not by matching the process name `ros2` -- the demo
# this script records (started separately, left running) is ALSO a `ros2`
# process, and an `-o comm` match on the bare name would false-positive on it.
if kill -0 "$recorder_pid" 2>/dev/null; then
  echo "WARNING: recorder pid ${recorder_pid} is still running after kill -INT" >&2
fi

if [ ! -f "${out_dir}/metadata.yaml" ]; then
  echo "WARNING: ${out_dir}/metadata.yaml is missing -- SIGINT may not have" >&2
  echo "finalized the bag cleanly" >&2
fi

echo "done: ${out_dir}"
exit "$episode_runner_status"
