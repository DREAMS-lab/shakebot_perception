# shakebot_perception
The perception ros package for shakebot. This package is made to estimate and verify the pose and acceleration of the shake table. It will provide us the data to know the ground truth motion of the shakebot. The pose will be measured from apriltag and the acceleration will be measured from Imu.

- The camera used for this package is "Chameleon3 with model number CM3-U3-13Y3C-CS".
- The Imu sensor used for this package is "Wit Motion HWT905-TTL" accelerometer or inclinometer sensor.

This perception package will require high computation for smooth processing so it is recommended to use a laptop or some high processing embedded CPU. We have already tested on raspberry pi 4 model b and it was lagging.

The accelerometer can be purchase from [this link](https://www.amazon.com/dp/B07G21XRV6/ref=emc_b_5_t)

Moreover, one can find all the details regarding accelerometer [here](https://github.com/WITMOTION/HWT905-TTL)

## Getting Started
Follow the below steps for environment setup
```
mkdir -p ~/catkin_ws/src    # make a new workspace (or you can clone in already present workspace)
cd ~/catkin_ws/src          # navigate to the source space
git clone https://github.com/AprilRobotics/apriltag.git      # Clone Apriltag library
git clone https://github.com/AprilRobotics/apriltag_ros.git  # Clone Apriltag ROS wrapper
git clone https://github.com/DREAMS-lab/shakebot_perception.git  # Clone shakbot_perception package
cd ~/catkin_ws              # Navigate to the workspace
rosdep install --from-paths src --ignore-src -r -y  # install any missing packages
catkin_make_isolated    # Build all packages in workspace (catkin build will also work)
```
After setting up the environment its time for camera calibration as its needed to detect apriltag's pose. The reference for monocular camera calibration can be found [here](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration).

To publish the camera images over ROS use the following command
```
roslaunch pointgrey_camera_driver camera.launch
```
> **_Note:_** We have used chameleon3 usb camera by point grey but if you are usign some different camera please contact your camera vendor for its respective camera drivers.

Open another terminal and type following command to start calibration. Please use [this pdf](assets/calib_io_checker_210x297_11x9_20.pdf) for calibration. Moreover, while printing, keep in mind to print it in actual size.
```
rosrun camera_calibration cameracalibrator.py --size 10x8 --square 0.020 image:=/camera/image_raw camera:=/camera
```
follow the calibration process mentioned [here](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)

## Quickstart
We have used apriltag_ros package for apriltag detection. The inputs for this package are `/camera/imge_rect`and `/camera/camera_info` topics. However, the outputs will be `/tf`, `/tag_detections` and `/tag_detections_image` topics.

one needs to print apriltags which can be found [here](assets/apriltag-imgs)

This package has following 3 launch files

1. `camera_detection.launch`: It will launch nodes that will only detect the apriltags and publish its pose.
2. `imu.launch`: It will launch nodes to publish data acquired from Imu.
3. `shakebot_perception.launch`: It will launch nodes that will function as combined of both above mentioned launch files.

User can run as per requirement. The command to run any launch file is
```
roslaunch shakebot_perception <launch file>

roslaunch shakebot_perception shakebot_perception.launch     # example to launch shakebot_preception.launch file
```

In order to detect the apriltags, one needs to enter tag details to [tags.yaml](/config/tags.yaml)
## Visualization
One can visualize the pose of apriltag and Imu captured by camera and Imu sensor respectively in Rviz.

this can be done by following command which will open up rviz 
```
rosrun rviz rviz
```

Things to configure in rviz for apriltag pose visualization:
- select the fixed frame as "camera".
- click on "add" button and select "TF" from the list.

Now one can try to detect the apriltag and can visualize its position and orientation in rviz.

Things to configure in rviz for Imu visualization:
- select the fixed frame as "HWT905".
- click on "add" button , go to by topic and select "imu" from the list.

> **_Note:_** View each data individually. Both camera and Imu data cannot be viewed at same time.