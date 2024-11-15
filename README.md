# UR5
# UR5 Kinematics and Joint Control

This project demonstrates how to compute forward kinematics for a UR5 robot and format the resulting transformation matrices to display each value with one decimal place. The code uses a forward kinematics function to calculate the position and orientation of the UR5’s end effector given the joint angles. The transformation matrix is formatted for easier readability, ensuring that all values are consistently displayed with one decimal point.

## Table of Contents
1. [Project Overview](#project-overview)
2. [Code Usage](#code-usage)
3. [License](#license)

## Project Overview

This project uses the **UR5 robot** and computes the **forward kinematics** using **Denavit-Hartenberg parameters** (DH parameters). The forward kinematics function calculates the position and orientation of the robot's end effector in space based on the joint angles. The resulting transformation matrix is then formatted to display each value with a single decimal place for better readability.

The transformation matrix is a 4x4 matrix that includes both the rotational and translational components of the robot’s end effector relative to its base.



### DH Parameters:
- **theta**: Joint angle (rotation)
- **d**: Link offset (translation along the z-axis)
- **a**: Link length (translation along the x-axis)
- **alpha**: Link twist (rotation about the x-axis)

These parameters are predefined in the `DH_PARAMS` array. The forward kinematics function multiplies individual transformation matrices for each joint to obtain the final transformation matrix of the end effector.

```python
import numpy as np

def dh_transform(theta, d, a, alpha):
    """Calculate individual transformation matrix using DH parameters."""
    # Construct the transformation matrix using DH parameters
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])
```
## Code Usage

### Dependecies
```bash
sudo apt-get update
sudo apt-get install python3-tk
sudo apt install -y python3-rosinstall python3-rosinstall-generator python3-wstool build-essential git
sudo apt install python3-rosdep
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-gazebo-ros2-control-demos
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
```
Extract and place the `ros2_ws` folder in home location.
```bash
cd ~/ros2_ws/
rosdep update
rosdep install -y --from-paths src --ignore-src -r
colcon build
source ~/ros2_ws/install/setup.bash
```

Run the simulation <br>
Terminal - 1
```bash
cd ~/ros2_ws/
source ~/ros2_ws/install/setup.bash
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:=ur5
```
Terminal - 2

```bash
cd ~/ros2_ws/
source ~/ros2_ws/install/setup.bash
python3 move.py
```

### Output
![Screenshot from 2024-11-16 04-24-48](https://github.com/user-attachments/assets/a64538f0-6fd4-4a65-a0ff-de180b169edf)


In the control Window, <br>
`+` : change in position in +direction <br>
`-` : change in position in -direction <br>

Use the `Home`, `Pick`, and `Place` button's for predefiend movement. <br>

Add the coordinates `x`, `y`, `z` for custom movement to that coordinate in space.
**Note**: There may be stuck in movement sometimes bcz, of inaccurate position calculation, for this go back to home location and rechange the coordinates to destination. <br>
Do not cross the reach limit of UR5
