# LoCO Description

This package contains information on the modeling and visualization of LoCO, with capabilities to visualize the robot in RViz and parse the URDF file for validating changes to the model.

## System Setup(s) Verified

- ROS: Melodic, Noetic
- Python: 3
- C++

## Package Downloads

To install these packages, use the command `sudo apt install ros-ros_version-insert_package_name`. After each new package download, run the command `rospack profile`.

- joint-state-publisher-gui

To install these packages, use the command `sudo apt install insert_package_name`. After each new package download, run the command `rospack profile`.

- liburdfdom-tools


## Environment Setup

Since development of the simulation resides in various packages and there is not a workspace available for download, a workspace must first be created. Instructions for this can be found at `http://wiki.ros.org/catkin/Tutorials/create_a_workspace`. It is recommended to name the workspace 'loco_ws' rather than 'catkin_ws' for keeping the workspace identifiable.

This is the only package required to model and view LoCO in RViz. For reference, the repository below should be cloned and located under the 'loco_ws/src/' path.

- loco_description: `https://github.umn.edu/loco-auv/loco_description` (though you are already here if you are reading this)

Once these packages have been installed, navigate back to the 'loco_ws' folder and enter the command `catkin_make` to finish configuring the workspace.

### If Using ROS Melodic

With the launch of ROS Noetic, the appropriate compatibility changes have been made to support the use of Python 3. However, ROS Melodic defaults to using Python 2 and will not successfully run all files without a slight setup modification. This can be found at https://answers.ros.org/question/326226/importerror-dynamic-module-does-not-define-module-export-function-pyinit__tf2/, but the same instructions are given below. This must be performed in a workspace with no "build" or "devel" folder.

Install some prerequisites to use Python3 with ROS

`sudo apt update`

`sudo apt install python3-catkin-pkg-modules python3-rospkg-modules python3-empy`

Prepare catkin workspace

`mkdir -p ~/catkin_ws/src; cd ~/catkin_ws` (this step is not required if using an existing workspace)

`catkin_make`

`source devel/setup.bash`

`wstool init`

`wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v 0.6.5`

`wstool up`

`rosdep install --from-paths src --ignore-src -y -r`

Finally compile for Python 3

`catkin_make --cmake-args
            -DCMAKE_BUILD_TYPE=Release
            -DPYTHON_EXECUTABLE=/usr/bin/python3
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so`

## Operating Within the Workspace

To operate within the LoCO workspace, first, create a terminal window (or tab) and navigate to the 'loco_ws' folder and enter the command `catkin_make` to ensure the workspace is up to date (if `catkin_make` has not already been performed). Then, enter `source devel/setup.bash` to source the setup. Any time a new terminal window or tab is created to run something from this workspace, the `source devel/setup.bash` command should be performed first.

## Launch LoCO in RViz
`roslaunch loco_description loco_rviz.launch`

## Run the Parser Program
The parser program is included for optional development purpose of verifying any changes to the robot URDF model file are correclty formatted. However, since the parser program cannot read macros currently in the URDF file, a base URDF file must be generated first. To do this, enter the command `xacro ./src/loco_description/urdf/loco.urdf.xacro > ./src/loco_description/urdf/loco.urdf`. Then, the urdf file can be checked by the parser with:
`./devel/lib/loco_description/parser ./src/loco_description/urdf/loco.urdf`

### View URDF Map
Once the URDF file has been parsed successfully, the URDF map can be viewed:

`urdf_to_graphiz ./src/loco_description/urdf/loco.urdf`

`evince loco.pdf`
