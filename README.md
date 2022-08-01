# shakebot_perception
The perception ros package for shakebot. This package is made to estimate and verify the pose and acceleration of the shake table. It will prvoide us the data to know the ground truth value motion of the shakebot.

## Getting Started 
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
## Quickstart
This package has following 3 launch files

1. `camera_detection.launch`: It will launch nodes that will only detect the apriltags and publish its pose.
2. `imu.launch`: It will launch nodes to publish data acquired from Imu.
3. `shakebot_perception.launch`: It will launch nodes that will function as combined of both above mentioned launch files.

User can run as per requirement. The command to run this launch file is
```
roslaunch shakebot_perception <launch file>

roslaunch shakebot_perception shakebot_perception.launch     # example to launch shakebot_preception.launch file
```

