# ROVI: Robotics and Vision in MuJoCo

A minimal MuJoCo-based simulation environment for the ROVI (Robotics and Vision) course. This project provides a framework for students to implement computer vision methods and path planning algorithms taught in the lectures, using a simulated UR5e robot with a Hand-E gripper.

---

![Demo Video](media/clip_rrtconnect.mp4)



## Project Structure
### Key Folders Explained:
*   **`exercises/`**: Students will place files (e.g., `exercise_1.py`) in this directory for their assignments. Teaching Assistants can place their corresponding solution files here.
*   **`main.py`**: The core simulation runner.
*   **`robot.py`**: Contains the `Robot` class with methods like `move_l()`,`move_j()`, `ur_ctrl_qpos()`, `ur_set_qpos()`, and `get_current_q()`.
*   **`cam.py`**: Contains the `Camera` class with methods to `get_rgb()`, `get_depth()`, and `get_pointcloud()`.



## Prerequisites

Before you begin, ensure you have the following installed on your system:
*   **Python 3.10** This project has been tested on version 3.10 (but will most likely work on 3.10+)
*   `pip` (usually comes with Python)
*   `venv` module (usually included in standard Python 3.10+ library)
*   **OMPL (Open Motion Planning Library)**: this project uses the Python bindings exposed by `ompl==1.7.0`.


## Getting Started

Follow these steps to set up the development environment and install dependencies.

1.  **Clone the repository:**
    ```bash
    git clone https://gitlab.sdu.dk/wilb/rovi2026.git
    cd rovi2026
    ```

2.  **Create a virtual environment:** (Recommended to isolate dependencies)
    ```bash
    python3.10 -m venv venv
    ```

3.  **Activate the virtual environment:**
    *   On Linux/macOS:
        ```bash
        source venv/bin/activate
        ```
    *   On Windows (Command Prompt):
        ```bat
        venv\Scripts\activate.bat
        ```
        Your command prompt should now show `(venv)` at the beginning.

4.  **Install the required Python packages:**
    ```bash
    python3.10 -m pip install -r requirements.txt
    ```

    If you already installed a newer incompatible OMPL version, downgrade it inside the active virtual environment:
    ```bash
    python3.10 -m pip install --force-reinstall "ompl==1.7.0"
    ```

You are now ready to run the project!

## Usage

### Running an Exercise
The main simulation is run by executing `main.py`. By default, it uses `scene.xml` and runs `exercises/exercise_1.py`.

Run the default exercise:
```bash
(venv) $ python3 main.py
```

Run a different scene:
```bash
(venv) $ python3 main.py --scene_path scene_reachability.xml
```

Run a different exercise file:
```bash
(venv) $ python3 main.py --exercise exercises/exercise_2.py
```

Run a different scene and exercise file:
```bash
(venv) $ python3 main.py --scene_path scene_reachability.xml --exercise exercises/exercise_2.py
```

Disable the physics-based position controller:
```bash
(venv) $ python3 main.py --disable_position_controller
```

Disable the TCP trail visualization:
```bash
(venv) $ python3 main.py --disable_tcp_trail
```


### Developing Your Solution
1.  Create a new file in the `exercises/` folder (e.g., `exercise_#.py`).
2.  Implement your `program(d, m)` function in that file. This is the entry point that `main.py` will call.
3.  Use the helper functions from `robot.py` and `cam.py` to control the robot and process vision data.
4.  Run your exercise with `python3 main.py --exercise exercises/exercise_#.py`.


## Getting Help

If you encounter issues, please check the following:
1.  Ensure all **Prerequisites** are met, especially the pinned Python installation of **OMPL**.
2.  Ensure your virtual environment is activated and all packages from `requirements.txt` are installed correctly.
3.  For course-specific questions, please refer to the course material or contact the teaching staff.
