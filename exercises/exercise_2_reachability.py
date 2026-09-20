"""Solution for exercise 3: reachability analysis for grasp candidates."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm

from utils.robot import UR5robot, get_mjobj_frame, ur_get_qpos


Command = tuple[np.ndarray, int | float | None]


def is_valid_configuration(
    robot: rtb.DHRobot,
    q: Sequence[float],
    floor_padding: float = 0.1,
) -> bool:
    """Return whether all robot link frames are above ``floor_padding``."""
    transforms = robot.fkine_all(q)
    return all(transform.t[2] >= floor_padding for transform in transforms[1:])


def plot_reachability(results_data: list[tuple[str, int]]) -> None:
    """Plot grasp success counts as a 2-by-5 heatmap."""
    values = np.array([value for _, value in results_data]).reshape(2, 5)
    print("Reachability values:", values)

    plt.imshow(values, cmap="viridis", interpolation="nearest")
    plt.colorbar(label="Valid grasp count")
    plt.title("Reachability Analysis")
    plt.xlabel("Cylinder column")
    plt.ylabel("Cylinder row")
    plt.savefig("ReachabilityAnalysis.png")


def program(d: Any, m: Any) -> list[Command]:
    """Run the reachability analysis and return poses for visualization."""
    robot = UR5robot(data=d, model=m)
    results_data: list[tuple[str, int]] = []
    vis_traj: list[Command] = []

    for cylinder_num in range(1, 11):
        cylinder_name = f"cylinder{cylinder_num}"
        cylinder_grasp_pose = (
            get_mjobj_frame(model=m, data=d, obj_name=cylinder_name) * sm.SE3.Tz(0.05)
        )
        success_count = 0

        for theta in np.linspace(0, 2 * np.pi, 100):
            print(f"Trying {cylinder_name} with theta={theta:.3f}")
            candidate_pose = cylinder_grasp_pose * sm.SE3.Rz(theta) * sm.SE3.Rx(
                -np.pi / 2
            )

            for _ in range(5):
                result = robot.robot_ur5.ik_LM(
                    Tep=candidate_pose,
                    q0=ur_get_qpos(d, m),
                )
                if not result[1]:
                    continue

                q_desired = result[0]
                if is_valid_configuration(robot.robot_ur5, q_desired):
                    vis_traj.append((q_desired, None))
                    success_count += 1
                    break

        results_data.append((cylinder_name, success_count))

    print("results_data:", results_data)
    plot_reachability(results_data)
    return vis_traj
