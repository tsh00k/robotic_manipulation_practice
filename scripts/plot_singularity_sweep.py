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

"""Plot the Stage M local DLS response along a near-singular TCP path."""

import csv
import sys

import matplotlib.pyplot as plt


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: plot_singularity_sweep.py INPUT_CSV OUTPUT_PNG")

    with open(sys.argv[1], newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    fraction = [float(row["fraction"]) for row in rows]
    sigma = [float(row["minimum_singular_value"]) for row in rows]
    damping = [float(row["damping"]) for row in rows]
    condition = [float(row["condition_number"]) for row in rows]
    joint_step = [float(row["maximum_joint_step"]) for row in rows]
    tracking = [float(row["linearized_tracking_error"]) for row in rows]

    figure, axes = plt.subplots(3, 1, figsize=(8, 9), sharex=True)
    axes[0].plot(fraction, sigma, label="minimum singular value")
    axes[0].plot(fraction, damping, label="adaptive damping")
    axes[0].set_ylabel("weighted value")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].semilogy(fraction, condition, label="condition number")
    axes[1].set_ylabel("condition number")
    axes[1].grid(True)

    axes[2].semilogy(fraction, tracking, label="linearized tracking error")
    axes[2].semilogy(fraction, joint_step, label="max |dq| [rad]")
    axes[2].set_xlabel("Cartesian path fraction")
    axes[2].set_ylabel("error / step")
    axes[2].legend()
    axes[2].grid(True)
    figure.suptitle("Stage M: weighted DLS near a singular configuration")
    figure.tight_layout()
    figure.savefig(sys.argv[2], dpi=160)


if __name__ == "__main__":
    main()
