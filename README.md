<!-- # Lane_Detection_with_Control_Steering -->
# Visual Odometry from Scratch

**Author**: Natthaphat Soookpanya (65340500023)  
**Framework**: ROS 2 + Gazebo + OpenCV (Python)

## 🛠️ Install My Project

### Dependencies
- Install `cv bridge`, `opencv`, `numpy`, `yaml`, `matplotlib`
    ```bash
    sudo apt install ros-humble-cv-bridge ros-humble-image-transport ros-humble-message-filters
    pip install opencv-python numpy pyyaml matplotlib
    ```
### Package
- Clone my package
    ```bash
    cd
    git clone <url>
    cd <ws name>
    colcon build && source install/setup.bash
    ```
## ⚙️ How to run code

1. Launch [rover_bringup.launch.py](src/rover_bringup/launch/rover_bringup.launch.py) for set environment. (Environment from LAB 1)
    ```bash
    ros2 launch rover_bringup rover_bringup.launch.py
    ```

2. Run node [visual_odom_orb_bf.pt](src/rover_visual_odom/scripts/visual_odom_orb_bf.py) for create visual odometry and see it in Rviz.
    ```bash
    ros2 run rover_visual_odom visual_odom_orb_bf.py
    ```

3. Run node [controller_server.py](src/rover_controller/scripts/controller_server.py) to make the robot tracking the path from LAB 1.
    ```bash
    ros2 run rover_controller controller_server.py
    ```

---

## 📌 Project Description

This project aims to develop a **Visual Odometry (VO)** system from scratch using **stereo cameras** on a mobile robot in a **real-time** simulation environment. The main goal is to implement VO using Python and OpenCV within a **ROS 2** and **Gazebo** simulation.

Key concepts involved:
- Transformation Matrix
- Stereo Depth Calculation
- Feature Detection & Matching (SIFT or ORB)

---

## 🎯 Objectives

1. Develop and study VO using Python and OpenCV from stereo camera input.
2. Deploy and test VO in a simulated mobile robot within Gazebo.
3. Use ROS 2 as the core framework for robot-camera-processing communication.
4. Ensure the VO system runs in **real-time** with stereo camera input.

---

## ✅ Requirements

### Functional Requirements

- **Stereo Camera Simulation in Gazebo**
  - Simulate left-right stereo cameras on a mobile robot.
  - Publish camera feeds to ROS topics.

- **Image Acquisition in ROS**
  - Subscribe to stereo image topics via ROS 2.
  - Use `cv_bridge` to convert ROS images for OpenCV use.

- **Visual Odometry Processing**
  - Detect and match features (e.g., ORB or SIFT).
  - Calculate Essential Matrix and Pose (Rotation, Translation).
  - Estimate robot position frame-to-frame.

- **Visualization**
  - Display robot trajectory in real-time (e.g., RViz or Matplotlib).
  - Compare VO trajectory to ground truth from Gazebo.

- **ROS Integration**
  - VO node must work within the ROS 2 ecosystem and interfaces.

### Performance Requirements

- VO processing rate: **≥ 10–15 Hz**.
- VO trajectory error: **≤ 10–15%** compared to ground truth.

---

## 🔍 Scope

1. Simulate a mobile robot with stereo camera in Gazebo.
2. Capture stereo image data using ROS 2 `image_raw` topics.
3. Compute VO using OpenCV:
   - Feature Detection & Matching
   - Essential Matrix Estimation
   - Pose Estimation (R, T)
   - Frame-to-frame motion tracking
4. Compare VO path with Gazebo's ground truth.

---

## 🧠 System Architecture

![system_overview](image/system_overview.png)

### 1. Camera Input from Gazebo
Uses the `libgazebo_ros_camera.so` plugin to simulate camera input in Gazebo. ROS image messages are converted to OpenCV format for processing.

### 2. Stereo to Depth Calculation
Uses disparity between left and right images to compute depth:

```math
Z = \frac{f * b}{x_L - x_R}
```

Where:
- $Z=$ depth
- $f=$ focal length
- $b=$ baseline (distance between cameras)
- $x_L,x_R=$ pixel coordinates from left and right images (disparity)

### 3. Feature Detection & Matching

#### Steps:
- **Feature Extraction**: Extract keypoints and descriptors from current and next left frames.
- **Feature Matching**: Match descriptors using a matcher (e.g., BFMatcher).
- **Match Filtering**: Filter poor matches using distance threshold.

### 4. Motion Estimation
Estimate robot motion (Odometry) using matched keypoints and depth information from stereo images.

## 🧑‍💻 Implementation

### Set Environment

### Set Robot

### Set Camera

### Setup Stereo Image Synchronization
```python
left_img_sub = Subscriber(self, Image, '/camera/left_image')
right_img_sub = Subscriber(self, Image, '/camera/right_image')
self.stereo_sync = ApproximateTimeSynchronizer([left_img_sub, right_img_sub], 10, 0.01)
```
Ensures both stereo images arrive close in time.

### Convert and Preprocess Images
```python
left_img = self.bridge.imgmsg_to_cv2(left_msg, "bgr8")
left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
```
Converts ROS image to grayscale OpenCV format.

### Depth Map from Stereo
```python
disparity = stereo_matcher.compute(left_gray, right_gray)
depth_map = (fx * baseline) / disparity
```
Uses `cv2.StereoSGBM_create()` to compute disparity, then calculates depth.

### ORB Feature Detection & Matching
```python
kp1, des1 = orb.detectAndCompute(prev_img, mask)
kp2, des2 = orb.detectAndCompute(curr_img, mask)
matches = bf_matcher.knnMatch(des1, des2, k=2)
```
Uses ORB + BruteForce with Lowe’s ratio test to match keypoints.

### Get 3D Points from Depth Map
Before estimating motion using PnP, we must convert 2D image keypoints (from the previous frame) to 3D coordinates using the depth map:
```python
fx = camera_params['fx']
fy = camera_params['fy']
cx = camera_params['cx']
cy = camera_params['cy']
```
I use the **pinhole camera model:**
```math
X = \frac{(u-c_x)*Z}{f_x},
Y = \frac{(v-c_y)*Z}{f_y},
Z = depthmap[v,u]
```

Where:
- $u=$ Pixel x-coordinate (horizontal) of the keypoint in the image
- $v=$ 	Pixel y-coordinate (vertical) of the keypoint in the image
- $c_x=$ Principal point x-coordinate, from the camera intrinsic matrix (K)
- $c_y=$ Principal point y-coordinate, from the camera intrinsic matrix (K)
- $f_x=$ Focal length in the x-direction (in pixels), from the intrinsic matrix
- $f_y=$ 	Focal length in the y-direction (in pixels), from the intrinsic matrix
- $Z=$ 	Depth value at pixel (u, v), retrieved from the depth map
- $X,Y=$ Reconstructed 3D point coordinates in the camera coordinate frame

**Camera Intrinsic Matrix (K):**

The matrix K (from `/camera/left_info`) looks like:

$$
\mathbf{K} = 
\begin{bmatrix}
f_x & 0 & c_x 
\\ 0 & f_y & c_y
\\ 0 & 0 & 1
\end{bmatrix}
$$

### Motion Estimation (PnP + RANSAC)
```python
success, rvec, tvec, inliers = cv2.solvePnPRansac(object_points, image_points, K, None)
R, _ = cv2.Rodrigues(rvec)
```
Estimates camera motion between frames using matched keypoints with known 3D depth.


### Pose Accumulation and Odometry Publishing
```python
T_cumulative = T_cumulative @ T_frame
```
Updates global transformation and publishes `/odometry/visual_odom`.
```python
odom_msg.pose.pose.position.x = position[0]
odom_msg.pose.pose.orientation = Quaternion(x, y, z, w)
```
Publishes robot's pose based on estimated motion.

## 📊 Result

## 🏁 Conclusion