import mujoco
import numpy as np
import spatialmath as sm
import time


DELAY_SECONDS = 3  # Start delay
DEFAULT_UR_POS = np.array([0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0])  # UR home position
REACHABLE_POINT_WORKSPACE = np.array([
    [0.25, 0.85],
    [-0.45, 0.45],
    [0.18, 0.55],
])


# NOTE: Visulization of the TCP trajectory
VISUALIZE_TCP_TRAIL = True
TCP_TRAIL_SITE_NAMES = ("tcp", "UR_TCP")
TCP_TRAIL_MAX_POINTS = 500
TCP_TRAIL_MIN_DISTANCE = 0.004
TCP_TRAIL_RADIUS = 0.0025
TCP_TRAIL_RGBA = np.array([0.0, 0.55, 1.0, 0.8])


def get_tcp_site_id(model):
    for site_name in TCP_TRAIL_SITE_NAMES:
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if site_id != -1:
            return site_id
    print(f"[WARN] Could not find a TCP site named one of {TCP_TRAIL_SITE_NAMES}")
    return -1


def update_tcp_trail(data, viewer, tcp_site_id, tcp_trail):
    if tcp_site_id == -1 or viewer.user_scn is None:
        return

    tcp_pos = data.site_xpos[tcp_site_id].copy()
    if not tcp_trail or np.linalg.norm(tcp_pos - tcp_trail[-1]) >= TCP_TRAIL_MIN_DISTANCE:
        tcp_trail.append(tcp_pos)
        if len(tcp_trail) > TCP_TRAIL_MAX_POINTS:
            del tcp_trail[:len(tcp_trail) - TCP_TRAIL_MAX_POINTS]

    points = tcp_trail[-min(len(tcp_trail), len(viewer.user_scn.geoms)):]
    viewer.user_scn.ngeom = 0
    for i, point in enumerate(points):
        rgba = TCP_TRAIL_RGBA.copy()
        rgba[3] *= 0.25 + 0.75 * (i + 1) / len(points)
        mujoco.mjv_initGeom(
            viewer.user_scn.geoms[i],
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[TCP_TRAIL_RADIUS, 0, 0],
            pos=point,
            mat=np.eye(3).flatten(),
            rgba=rgba,
        )
    viewer.user_scn.ngeom = len(points)

def spawn_reachable_points(
    data,
    model,
    robot,
    count=6,
    marker_prefix="random_point",
    workspace=REACHABLE_POINT_WORKSPACE,
    seed=None,
    max_attempts=1000,
):
    rng = np.random.default_rng(seed)
    workspace = np.asarray(workspace, dtype=float)
    if workspace.shape != (3, 2):
        raise ValueError("workspace must have shape (3, 2)")

    marker_body_ids = []
    for i in range(count):
        marker_name = f"{marker_prefix}_{i + 1}"
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, marker_name)
        if body_id == -1:
            raise ValueError(f"Scene is missing marker body '{marker_name}'")
        marker_body_ids.append(body_id)

    q_seed = robot.get_current_q()
    point_frames = []
    for i, body_id in enumerate(marker_body_ids):
        for _ in range(max_attempts):
            point = rng.uniform(workspace[:, 0], workspace[:, 1])
            target_frame = sm.SE3.Trans(point) * sm.SE3.Rx(-np.pi)
            ik_result = robot.robot_ur5.ik_LM(Tep=target_frame, q0=q_seed)
            if not ik_result[1]:
                continue

            model.body_pos[body_id] = point
            q_seed = ik_result[0]
            point_frames.append(sm.SE3.Trans(point))
            break
        else:
            raise RuntimeError(f"Could not find reachable point {i + 1} after {max_attempts} attempts")

    mujoco.mj_forward(model, data)
    return point_frames

def initialize_default_state(data, model, viewer, ur_set_qpos, hande_ctrl_qpos):
    # NOTE: Initialize the robot to a default position on startup
    ur_set_qpos(data=data, q_desired=DEFAULT_UR_POS)
    hande_ctrl_qpos(data=data, gripper_value=0)  # Open gripper

    # NOTE: Add a 3 second delay before executing your script
    sim_start = time.time()
    while time.time() - sim_start < DELAY_SECONDS:
        mujoco.mj_step(model, data)
        viewer.sync()
    
