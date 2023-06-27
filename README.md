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

For simulation purposes, you can use the [Unitree ROS](https://github.com/unitreerobotics/unitree_ros) Git repository or  Clone this repository and run catkin_make, which contains the necessary files for simulations. Remember to put this in a workspace where you want to run from, i use this same repo as my catkin workspace so wherever they mention 'catkin_ws' it will be this workspace i use


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

## Contributing

Special thanks to [dbaldwin](https://github.com/dbaldwin) for their contributions and assistance in this project.

Contributions to this repository are welcome! If you have any improvements, bug fixes, or new features to suggest, please submit a pull request. Make sure to follow the repository's guidelines for contributing.

## License

This repository is licensed under the [MIT License](LICENSE). Feel free to use and modify the code according to your needs.
