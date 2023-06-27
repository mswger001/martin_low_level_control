Apologies for missing that step. Here's an updated version that includes modifying the CMakeLists.txt file:

To run the generalized scripts in your Unitree Controller workspace, follow these steps:

1. Open the CMakeLists.txt file located in your Unitree Controller workspace. You can find it at: `~/catkin_ws/src/unitree_controller/CMakeLists.txt`.

2.Modify the CMakeLists.txt file:

Open the CMakeLists.txt file located at ~/catkin_ws/src/unitree_controller/CMakeLists.txt.

Add the following lines to the CMakeLists.txt file under the add_executable() section:

   ```cmake
   catkin_install_python(PROGRAMS scripts/your_file.py
       DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )
   ```

3. Save the CMakeLists.txt file after adding the lines.

4. Build your Catkin workspace to apply the changes:

   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

5. Open a new terminal and source the `setup.bash` file:

   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```

6. Navigate to the directory where the generalized scripts are located:

   ```bash
   cd <path_to_generalized_scripts>
   ```

   Replace `<path_to_generalized_scripts>` with the actual path to the folder where the generalized scripts are located.

7. Copy the scripts to your Unitree Controller workspace:

   ```bash
   cp *.py ~/catkin_ws/src/unitree_controller/
   ```

8. Run the desired script using the `rosrun` command. For example, to run the `test_hip.py` script:

   ```bash
   rosrun unitree_controller test_hip.py
   ```

   Please note that if the script requires additional dependencies, ensure that they are installed before running the script.

That's it! You can now modify the CMakeLists.txt file to copy the generalized scripts and run them in your Unitree Controller workspace.