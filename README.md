# Martin Low-Level Control

This repository provides control for the Martin (Go1) robot at the low-level, both in simulation and in real implementation. It aims to bring together all the relevant documentation and resources in one place for easy reference.

## Contents

- [Documentation](#documentation)
- [Simulations](#simulations)
- [Setup](#setup)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Documentation

To understand the Martin robot and its functionalities, refer to the official [Unitree Go1 Documentation](https://docs.trossenrobotics.com/unitree_go1_docs/downloads.html). It provides comprehensive information about the robot's capabilities, features, and specifications.

Certainly! Here's a modified version of the "Simulations" section to improve readability:

## Simulations

For simulating the Martin robot, you have two options:

### Option 1: Using Unitree ROS Git Repository

You can use the Unitree ROS Git repository for simulation purposes. Follow the steps they give in he [Unitree ROS](https://github.com/unitreerobotics/unitree_ros) Git repository



### Option 2: Using this Repository

Alternatively, you can use this repository for simulation purposes. Follow these steps:

1. Make sure you have the following dependencies installed on your system:
   - ROS (Robot Operating System)
   - Gazebo
   - Unitree SDKs

2. Clone this repository to your local environment:

   ```bash
   git clone https://github.com/mswger001/martin_low_level_control.git

   git submodule update --init --recursive

   ```

3. Set up your Catkin workspace. If you don't have a Catkin workspace, create one with the desired name (e.g., `my_workspace`):

   ```bash
   mkdir -p ~/my_workspace/src
   cd ~/my_workspace/src
   catkin_init_workspace
   ```

4. Copy the necessary files from the cloned repository to your Catkin workspace:

   ```bash
   cp -r martin_low_level_control/* ~/my_workspace/src
   ```

5. If you have additional Python files to include in the ROS package, update the `CMakeLists.txt` file located at `~/my_workspace/src/unitree_ros/unitree_controller/CMakeLists.txt`. Use the `catkin_install_python` command to specify the path to the Python file you want to include:

   ```cmake
   catkin_install_python(PROGRAMS scripts/your_file.py
       DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )
   ```

6. Build your Catkin workspace:

   ```bash
   cd ~/my_workspace
   catkin_make
   ```

7. Source the `setup.bash` file to set up the environment:

   ```bash
   source ~/my_workspace/devel/setup.bash
   ```

Choose either option based on your preference for simulating the Martin robot. 


## Setup

To get started with the Martin low-level control, follow these steps:

1. Launch a terminal (based on your environment: Windows or Linux).

2. Set up the necessary environment for the Martin low-level control (based on your chosen option):

   - **Option 1: Linux Environment with ROS, Gazebo, and Unitree SDKs**
   
     - Create a catkin workspace and navigate to the `src` directory:
     
       ```bash
       mkdir -p catkin_ws/src
       cd catkin_ws/src
       ```

     - Clone the `unitree_ros` repository:
     
       ```bash
       git clone https://github.com/unitreerobotics/unitree_ros.git
       ```
       
     - Build the ROS workspace:
     
       ```bash
       cd ..
       catkin_make
       ```

   - **Option 2: Windows Environment with Docker (Recommended)**
   
     - Run the following command to start a Docker container for ROS with VNC support:
     
       ```bash
       docker run -p 6080:80 --shm-size=512m --name ros_go1 -d tiryoh/ros-desktop-vnc:melodic
       ```
       
     - Access the Docker instance through the provided URL in your web browser.
     
     - Once inside the Docker container's home directory, create a catkin workspace and navigate to the `src` directory:
     
       ```bash
       mkdir -p catkin_ws/src
       cd catkin_ws/src
       ```
     
   - **Shared Step for Both Options: Clone the Unitree Legged SDK for Go1**
   
     - Clone the Unitree Legged SDK for Go1 repository, specifically the `v3.8.0` branch:
   
       ```bash
       git clone -b v3.8.0 https://github.com/unitreerobotics/unitree_legged_sdk.git
       ```
       
     - Build the low-level components by navigating to the `build` directory within the `unitree_legged_sdk` folder:
   
       ```bash
       cd unitree_legged_sdk
       mkdir build
       cd build
       cmake ..
       make
       ```

3. Configure Go1 into low-level mode by following these button combinations:

   ```
   L2 + A
   L2 + A
   L2 + B
   L1 + L2 + Start
   ```

4. Suspend Go1 on a rack using the provided example:

   ```bash
   ./example_position
   ```

5. In the workspace directory (`~/catkin_ws/src` for Option 1 or `catkin_ws/src` for Option 2), clone the `unitree_ros_to_real` repository:

   ```bash
   git clone https://github.com/unitreerobotics/unitree_ros_to_real.git
   ```

6. Build the ROS workspace:

   ```bash
   cd ~/catkin_ws (for Option 1) or cd catkin_ws (for Option 2)
   catkin_make
   ```

7. Source the `setup.bash` file:

   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```

8. Proceed to usage instructions specific to the Martin low-level control below.



## Usage

Once you have completed the setup steps based on your environment, you can now use the Martin low-level control. Follow these steps:

1. Launch the low-level ROS node:

   ```bash
   roslaunch unitree_legged_real real.launch ctrl_level:=lowlevel
   ```

2. Open a new terminal and source the `setup.bash` file (or add it to `bashrc`):

   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```

3. Run the low-level example:

   ```bash
   rosrun unitree_legged_real example_position
   ```

By following these steps, you will be able to control the Martin robot in low-level mode using the provided examples and configurations.
Here's a combined explanation of using the Unitree SDK and the `LowCmd` structure and its parameters:

### Using Unitree SDK and Motor Commands

1. **Include SDK Header**: In your C++ code, include the Unitree SDK header file using the following statement: `#include "unitree_legged_sdk/unitree_legged_sdk.h"`.

2. **Initialize UDP Communication**: Set up UDP communication with the Unitree robot using the `UDP` class. Specify the robot's IP address and port number for sending and receiving data.

3. **Create a Custom Class**: Create a custom class to manage your robot's control. This class should include an instance of the `UDP` class for communication and a `LowCmd` object for sending motor commands.

4. **Robot Control Loop**: Implement a control loop where you define the desired behavior of the robot. Inside this loop:

   - Receive robot state data using `udp.GetRecv(state)` to get information about the current state of the robot.

   - Calculate and set desired motor commands, including joint positions (`q`), joint velocities (`dq`), proportional gains (`Kp`), derivative gains (`Kd`), and torques (`tau`) for each motor.

5. **Safety Checks**: Implement safety checks as needed, such as position limiting and power protection, to ensure the robot operates safely.

6. **Start Control Loop**: Start the control loop to continuously send motor commands to the robot.

7. **Main Function**: In the `main()` function:

   - Initialize the custom class, setting the communication level and other parameters.

   - Start control loops for sending and receiving UDP data to interact with the robot.

   - Enter a continuous loop to keep the program running and the control process active.

### LowCmd Structure and Parameters

The `LowCmd` structure is a crucial part of the Unitree SDK, as it allows you to send motor commands to control the robot's joints. It contains various parameters that define the desired state and behavior of each motor. Here's an explanation of some of the key parameters within the `LowCmd` structure:

1. **`motorCmd[]` Array**: This is an array of motor commands, typically one command for each joint of the robot. You can access individual motor commands using indices (e.g., `motorCmd[0]` for the first joint).

2. **`q` (Joint Position):** `motorCmd[].q` represents the desired joint position in radians. You set this parameter to specify where you want the joint to move to.

3. **`dq` (Joint Velocity):** `motorCmd[].dq` represents the desired joint velocity in radians per second. Setting this parameter controls how fast the joint should move.

4. **`Kp` (Proportional Gain):** `motorCmd[].Kp` is the proportional gain for the PD controller. It determines the strength of the position control. Higher values make the joint respond more aggressively to position errors.

5. **`Kd` (Derivative Gain):** `motorCmd[].Kd` is the derivative gain for the PD controller. It controls the damping effect on joint movement. Higher values provide more damping, which can help stabilize motion.

6. **`tau` (Torque):** `motorCmd[].tau` specifies the desired torque to be applied to the joint. You can use this parameter to apply external forces or torques to the joint.

7. **Other Parameters:** Depending on the specific robot and use case, there may be additional parameters in the `LowCmd` structure for more advanced control and configuration.

In summary, the `LowCmd` structure allows you to control the position (`q`), velocity (`dq`), stiffness (`Kp` and `Kd`), and torque (`tau`) of each joint in the robot. By setting these parameters appropriately, you can define the robot's desired behavior and movements.

Please note that the exact usage of these parameters may vary depending on the specific Unitree robot model and control strategy, so it's important to refer to the official Unitree SDK documentation and resources for precise details related to your robot's configuration and control requirements.

## Contributing

Special thanks to [dbaldwin](https://github.com/dbaldwin) for their contributions and assistance in this project.

Contributions to this repository are welcome! If you have any improvements, bug fixes, or new features to suggest, please submit a pull request. Make sure to follow the repository's guidelines for contributing.

## License

This repository is licensed under the [MIT License](LICENSE). Feel free to use and modify the code according to your needs.
