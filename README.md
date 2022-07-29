# shaker_percept
The perception ros package for shake table. This package is made to estimate and verify the pose and acceleration of the shake table.

## Quick Start 
Follow the below steps for environment setup
```
mkdir -p ~/catkin_ws/src    # make a new workspace (or you can clone in already present workspace)
cd ~/catkin_ws/src          # navigate to the source space
git clone https://github.com/AprilRobotics/apriltag.git      # Clone Apriltag library
git clone https://github.com/AprilRobotics/apriltag_ros.git  # Clone Apriltag ROS wrapper
git clone https://github.com/DREAMS-lab/shaker_percept.git  # Clone shake_percept package
cd ~/catkin_ws              # Navigate to the workspace
rosdep install --from-paths src --ignore-src -r -y  # install any missing packages
catkin_make_isolated    # Build all packages in workspace (catkin build will also work)
```
