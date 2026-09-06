#############################################
# ROVI: Robotics Exercises
# Supporting: Ubuntu 22-24, Python 3.10
#############################################

import argparse
import importlib.util
import queue
import time

import mujoco
import mujoco.viewer
import numpy as np

from utils.utils import *
# NOTE: Custom Robot and Camera Classes
from utils.robot import *
from utils.cam import *

# NOTE: True:   Utilises the physics-based position control of the UR-joints. (Needed for grasping objects)  
#       False:  Forcing a position, i.e., no dynamics. (Needed for Exercise 3)
USE_POSITION_CONTROLLER = True
USE_SHADOWS = False


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene_path", default="scene.xml")
    parser.add_argument("--exercise", default="exercises/exercise_1.py")
    parser.add_argument("--disable_position_controller", action="store_false", dest="use_position_controller", default=True)
    parser.add_argument("--disable_tcp_trail", action="store_false", dest="visualize_tcp_trail", default=True)
    return parser.parse_args()


def load_program(exercise_path):
    spec = importlib.util.spec_from_file_location("exercise", exercise_path)
    exercise = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(exercise)
    return exercise.program


def validate_command_queue(command_queue):
    if command_queue is None:
        return []

    if not isinstance(command_queue, list):
        raise TypeError("program(data, model) must return a list")

    for i, command in enumerate(command_queue):
        if not isinstance(command, tuple) or len(command) != 2:
            raise TypeError(f"Command {i} must be a tuple: (desired_cmd, gripper_value)")

        desired_cmd, gripper_value = command
        if not isinstance(desired_cmd, np.ndarray) or desired_cmd.shape != (6,):
            raise TypeError(f"Command {i} desired_cmd must be a numpy array with shape (6,)")

    return command_queue

if __name__ == "__main__":
    args = parse_args()
    program = load_program(args.exercise)

    # NOTE: scene_path specifies your .xml scene file
    # To change it either replace here or in the GUI
    # e.g. scene_path = "scene_project_cv.xml"
    scene_path = args.scene_path

    # NOTE: Initialize MuJoCo model and data
    model = mujoco.MjModel.from_xml_path(scene_path)
    data = mujoco.MjData(model)

    # NOTE: Queue used for keyboard input from the viewer
    key_queue = queue.Queue()
    command_queue = []
    tcp_site_id = get_tcp_site_id(model) if args.visualize_tcp_trail else -1
    tcp_trail = []

    # NOTE: Create the MuJoCo window loop
    with mujoco.viewer.launch_passive(
        model=model,
        data=data,
        key_callback=lambda key: key_queue.put(key),
    ) as viewer:
        viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = int(USE_SHADOWS)

        # NOTE: Default initialization script. Modify this as needed
        initialize_default_state(data=data, model=model, viewer=viewer, 
                                 ur_set_qpos=ur_set_qpos, hande_ctrl_qpos=hande_ctrl_qpos)

        # NOTE: Your code implementations will use the "program" interface
        command_queue = program(data, model)  # Run the program after the initial setup
        command_queue = validate_command_queue(command_queue)

        # NOTE: Simulation loop
        while viewer.is_running():
            # mj_step: does one simulation step
            mujoco.mj_step(model, data)

            # If any commands are given
            if command_queue:
                # Extract and remove the first command from the queue
                command, command_queue = command_queue[0], command_queue[1:]

                # NOTE: Each command should be a tuple of a robot command and a gripper command
                desired_cmd, gripper_value = command

                # Control the robot
                if isinstance(desired_cmd, np.ndarray):
                    if args.use_position_controller:
                        # Control the robot using the scene's position actuators.
                        ur_ctrl_qpos(data=data, q_desired=desired_cmd)
                    else:
                        # Set the robot pose directly. Useful for visualization.
                        ur_set_qpos(data=data, q_desired=desired_cmd)
                else:
                    print("[NOTE] desired_cmd: ", type(desired_cmd))

                # Control the gripper
                if gripper_value is not None:
                    hande_ctrl_qpos(data=data, gripper_value=gripper_value)

            if args.visualize_tcp_trail:
                with viewer.lock():
                    update_tcp_trail(data=data, viewer=viewer, tcp_site_id=tcp_site_id, tcp_trail=tcp_trail)

            # Pick up changes to the physics state, apply perturbations, update options from GUI.
            viewer.sync()
