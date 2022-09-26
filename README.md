# Shakebot_Perception
The perception ros package for shakebot. This package is made to estimate and verify the pose and acceleration of the shake table. It will provide us the data to know the ground truth motion of the shakebot. The pose will be measured from apriltag and the acceleration will be measured from Imu.

- The camera used for this package is "Chameleon3 with model number [CM3-U3-13Y3C-CS](https://www.edmundoptics.com/p/cm3-u3-13y3c-12-chameleon3-color-camera/2862/)", which has frame rate as high as 149 fps. 
- The Imu sensor used for this package is "[Wit Motion HWT905-TTL](https://www.amazon.com/dp/B07G21XRV6/ref=emc_b_5_t)" accelerometer or inclinometer sensor.

This perception package will require high computation for smooth processing so it is recommended to use a laptop or some high processing embedded CPU. We have already tested on raspberry pi 4 model b and it was lagging.

Moreover, one can find all the details regarding accelerometer [here](https://github.com/WITMOTION/HWT905-TTL)

## Getting Started
Follow the below steps for environment setup
```bash
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
```bash
roslaunch pointgrey_camera_driver camera.launch
```
> **_Note:_** We have used chameleon3 usb camera by point grey but if you are usign some different camera please contact your camera vendor for its respective camera drivers.

Open another terminal and type following command to start calibration. Please use [this pdf](assets/calib_io_checker_210x297_11x9_20.pdf) for calibration. Moreover, while printing, keep in mind to print it in actual size.
```bash
rosrun camera_calibration cameracalibrator.py --size 10x8 --square 0.020 image:=/camera/image_raw camera:=/camera
```
follow the calibration process mentioned [here](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)


## Environment Configuration

This configuration deals with communication setup between computer and shakebot, and camera frame rate settings.

To setup communication between Shakebot and computer first check if both are connected on same network. if not so do it. try to figure out the ip address of the computer(host) by following command. The computer will act as a host and the shakebot will be slave in this case.

```bash
ifconfig
```
after knowing you computer's ip address follow steps below to setup network configuration that allows communication.

```bash
echo 'export ROS_MASTER_URI=http://<host ip address>:11311/' >> ~/.bashrc
echo 'export ROS_HOSTNAME=<host ip address>' >> ~/.bashrc
echo 'export ROS_IP=<host ip address>' >> ~/.bashrc
source ~/.bashrc
```
follow the procedure to setup communication in shakebot manual for shakebot hardware configuration.

Now, for camera frame rate configuration open [camera_fps_config.yaml](./config/camera_fps_config.yaml) file. Find a parameter `frame_rate` and change the value according to the requirement. we kept its value as 100.

## Quickstart
We have used apriltag_ros package for apriltag detection. The inputs for this package are `/camera/imge_rect` and `/camera/camera_info` topics. However, the outputs will be `/tf`, `/tag_detections` and `/tag_detections_image` topics.

One needs to print apriltags which can be found [here](assets/apriltag-imgs).

In order to detect the apriltags, one needs to enter tag details to [tags.yaml](/config/tags.yaml).

This package has following 3 launch files

1. `camera_detection.launch`: It will launch nodes that will only detect the apriltags and publish its pose.
2. `imu.launch`: It will launch nodes to publish data acquired from Imu.
3. `shakebot_perception.launch`: It will launch nodes that will function as combined of both above mentioned launch files.

User can run as per requirement. The command to run any launch file is
```bash
roslaunch shakebot_perception <launch file>

roslaunch shakebot_perception shakebot_perception.launch     # example to launch shakebot_preception.launch file
```

## Start Recording data
To start recording the data start shakebot_perception launch file using the code below

```bash
roslaunch shakebot_perception shakebot_perception.launch
```

follow the onscreen instructions and after completion the data will be stored in [data folder](./data/) in json file, name starting with recorded and containing timestamp. for example, [recorded_09_07_22_10_48_10.json](./data/recorded_09_07_22_10_48_10.json) which is created on 9th september, 2022 at 10:48:10 am.

## Visualization (optional)
One can visualize the pose of apriltag and Imu captured by camera and Imu sensor respectively in Rviz.

this can be done by following command which will open up rviz 
```bash
rosrun rviz rviz
```

Things to configure in rviz for apriltag pose visualization:
- select the fixed frame as "camera".
- click on "add" button and select "TF" from the list.

Now one can try to detect the apriltag and can visualize its position and orientation in rviz.

Things to configure in rviz for Imu visualization:
- select the fixed frame as "HWT905".
- click on "add" button , go to by topic and select "imu" from the list.

> **_Note:_** View each data individually. Both camera and Imu data cannot be viewed at same time, because their parent frames are different.