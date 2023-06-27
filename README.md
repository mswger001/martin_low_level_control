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

## Simulations

For simulation purposes, you can use the [Unitree ROS](https://github.com/unitreerobotics/unitree_ros) Git repository or  Clone this repository and navigate to the `go1_ros_combined` directory, which contains the necessary files for simulations.

## Setup

To get started with the Martin low-level control, follow these steps:

1. Launch a Windows terminal.

2. Run the following command to start a Docker container for ROS (Robot Operating System) with VNC support:

   ```
   docker run -p 6080:80 --shm-size=512m --name ros_go1 -d tiryoh/ros-desktop-vnc:melodic
   ```

3. Access the Docker instance through the following URL in your web browser:

   ```
   http://localhost:6080
   ```

4. Once inside the Docker container's home directory, create a catkin workspace and navigate to the `src` directory:

   ```
   mkdir -p catkin_ws/src
   cd catkin_ws/src
   ```

5. Clone the Unitree Legged SDK for Go1 repository, specifically the `v3.8.0` branch, using the following command:

   ```
   git clone -b v3.8.0 https://github.com/unitreerobotics/unitree_legged_sdk
   ```

6. Build the low-level components by navigating to the `build` directory within the `unitree_legged_sdk` folder:

   ```
   cd unitree_legged_sdk
   mkdir build
   cd build
   cmake ..
   make
   ```

7. Configure Go1 into low-level mode by following these button combinations:

   ```
   L2 + A
   L2 + A
   L2 + B
   L1 + L2 + Start
   ```

8. Suspend Go1 on a rack using the provided example:

   ```
   ./example_position
   ```

9. Now, let's set up the ROS low-level example. In the Docker container's home directory, clone the `unitree_ros_to_real` repository:

   ```
   cd ~/catkin_ws/src
   git clone https://github.com/unitreerobotics/unitree_ros_to_real
   ```

10. Build the ROS workspace:

    ```
    cd ~/catkin_ws
    catkin_make
    ```

11. Source the `setup.bash` file:

    ```
    source ~/catkin_ws/devel/setup.bash
    ```

## Usage

To run the low-level control example for Martin, follow these steps:

1. Launch the low-level ROS node:

   ```
   roslaunch unitree_legged_real real.launch ctrl_level:=lowlevel
   ```

2. Open a new terminal and source the `setup.bash` file (or add it to your `bashrc`):

   ```
   source ~/cat

kin_ws/devel/setup.bash
   ```

3. Run the low-level control example:

   ```
   rosrun unitree_legged_real example_position
   ```

## Contributing

Special thanks to [dbaldwin](https://github.com/dbaldwin) for their contributions and assistance in this project.

Contributions to this repository are welcome! If you have any improvements, bug fixes, or new features to suggest, please submit a pull request. Make sure to follow the repository's guidelines for contributing.

## License

This repository is licensed under the [MIT License](LICENSE). Feel free to use and modify the code according to your needs.
