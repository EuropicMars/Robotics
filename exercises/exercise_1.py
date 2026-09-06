import numpy as np
import spatialmath as sm

from utils.robot import UR5robot, get_mjobj_frame, is_q_valid, ur_get_qpos


def program(d, m):
    """Example structure for a ROVI exercise solution.

    main.py calls this function once after the MuJoCo scene has been loaded.
    The function should return a command queue:

        [(q_desired, gripper_value), ...]

    where q_desired is a numpy array with 6 UR joint values and
    gripper_value is a number or None.
    """
    robot = UR5robot(data=d, model=m)

    # Read the current robot joint configuration.
    current_q = robot.get_current_q()
    # This standalone helper returns the same type of 6-joint vector.
    current_q_from_helper = ur_get_qpos(data=d, model=m)

    print("Current joint values:", current_q)
    print("Current joint values from helper:", current_q_from_helper)

    # Read the current tool center point (TCP) pose using the Robotics Toolbox
    # model inside UR5robot.
    current_tcp = robot.get_current_tcp()
    print("Current TCP pose:")
    print(current_tcp)

    # Example 1: move_j makes a joint-space motion to the box.
    #
    # You give move_j two 6D joint vectors: start_q and end_q.
    # The robot interpolates each joint angle from start to end.
    # This is simple and usually fast, but the TCP may move in a curved path.
    box_frame = get_mjobj_frame(model=m, data=d, obj_name="box")
    grasp_frame = box_frame * sm.SE3.Tz(0.1) * sm.SE3.Rx(np.pi)
    box_q, box_success, _, _, _ = robot.robot_ur5.ik_LM(Tep=grasp_frame, q0=current_q)

    if box_success and is_q_valid(d=d, m=m, q=box_q):
        robot.move_j(start_q=current_q, end_q=box_q, t=500)

        # Example: open/close the Hand-E gripper.
        # set_gripper repeats the last robot pose while changing the gripper value.
        # Use 0 for open and 255 for closed.
        robot.set_gripper(value=255, t=100)
        robot.set_gripper(value=0, t=100)

        # Reset after move_j.
        #
        # This is also a joint-space move: the robot returns from the box posture
        # to the original starting joint configuration.
        robot.move_j(start_q=box_q, end_q=current_q, t=500)

    # Example 2: move_l makes a Cartesian straight-line motion to the cylinder.
    #
    # You give move_l two TCP poses: T0 and T1.
    # The robot interpolates the TCP pose in Cartesian space and solves IK at
    # each step. This is useful when the end-effector should move straight
    # toward or away from an object.
    cylinder_frame = get_mjobj_frame(model=m, data=d, obj_name="cylinder")
    cylinder_tcp = cylinder_frame * sm.SE3.Tz(0.15) * sm.SE3.Rx(np.pi)
    cylinder_q, cylinder_success, _, _, _ = robot.robot_ur5.ik_LM(
        Tep=cylinder_tcp,
        q0=current_q,
    )

    if cylinder_success and is_q_valid(d=d, m=m, q=cylinder_q):
        robot.move_l(T0=current_tcp, T1=cylinder_tcp, q0=current_q, total_time=1.0)

    return robot.queue
