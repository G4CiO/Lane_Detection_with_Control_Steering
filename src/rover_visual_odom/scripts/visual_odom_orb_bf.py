#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
import time
import os
from threading import Lock
from collections import deque
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
from tf_transformations import quaternion_from_matrix, euler_matrix

try:
    from message_filters import ApproximateTimeSynchronizer, Subscriber
except ImportError:
    # Fallback if message_filters not available
    ApproximateTimeSynchronizer = None
    Subscriber = None


class OptimizedStereoVO(Node):
    def __init__(self):
        super().__init__('optimized_stereo_vo')
        self.state = 0 # 0: recording, 1: finished
        # Declare parameters for easy tuning
        self.setup_parameters()
        
        # Synchronized subscribers for stereo images
        self.setup_subscribers()

        # Setup recording for disparity and depth images
        self.setup_record_dis_and_depth()
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odometry/visual_odom', 10)
        
        # Core components
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.lock = Lock()
        
        # Camera parameters
        self.camera_params = {}
        self.baseline = 0.05  # 5cm baseline
        
        # VO state
        self.T_cumulative = np.eye(4)
        self.prev_frame = None
        self.frame_history = deque(maxlen=5)  # For temporal consistency
        
        # Feature detector and matcher
        self.setup_feature_detector()
        
        # Stereo matcher
        self.setup_stereo_matcher()
        
        self.get_logger().info("Optimized Stereo VO initialized with ORB features")

    def setup_parameters(self):
        """Declare ROS parameters for easy tuning"""
        # ORB feature detection parameters
        self.declare_parameter('max_features', 1000)
        self.declare_parameter('scale_factor', 1.2)
        self.declare_parameter('n_levels', 8)
        self.declare_parameter('edge_threshold', 31)
        self.declare_parameter('first_level', 0)
        self.declare_parameter('wta_k', 2)
        self.declare_parameter('patch_size', 31)
        self.declare_parameter('fast_threshold', 20)
        
        # BruteForce matcher parameters
        self.declare_parameter('bf_cross_check', False)
        self.declare_parameter('lowe_ratio', 0.3)
        
        # Stereo matching parameters
        self.declare_parameter('num_disparities', 96)
        self.declare_parameter('block_size', 11)
        
        # Motion estimation parameters
        self.declare_parameter('max_depth', 500.0)
        self.declare_parameter('min_depth', 0.0)
        self.declare_parameter('ransac_threshold', 1.0)
        self.declare_parameter('min_matches', 4)

        # Offset parameters for robot frame
        self.declare_parameter('offset_x', 6.683857497627434)
        self.declare_parameter('offset_y', 3.509575305683288)
        self.declare_parameter('offset_z', 0.0)
        self.declare_parameter('offset_yaw', 2.6103058788899838)
        
        # RGB to grayscale conversion
        self.declare_parameter('rgb', True)  # Set to False if images are already grayscale

        # Check for reasonable motion (not too large jumps)
        self.declare_parameter('max_translation', 4.0)  # 2 meters per frame is unrealistic

        # Visualization
        # self.declare_parameter('enable_matches_visualization', False)
        self.declare_parameter('record_matches', False)
        self.declare_parameter('record_disparity_depth_left_image', False)

        # TF publishing
        self.declare_parameter('publish_tf', False)

    def setup_record_dis_and_depth(self):
        self.recording_started = False
        self.recording_duration = 120  # seconds
        self.record_start_time = None

        # Define output video settings
        self.fps = 30
        self.frame_size = (1384, 1032)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')

        # Output directory
        # pkg_path = os.path.dirname(os.path.realpath(__file__))
        # save_path = os.path.join(pkg_path, '..', 'video_output')
        # os.makedirs(save_path, exist_ok=True)
        self.save_path = os.path.join(os.getcwd(), 'video_output')
        os.makedirs(self.save_path, exist_ok=True)
        # self.save_path = os.path.join(data_dir, file_name)

        # VideoWriters (initialized when recording starts)
        self.disparity_writer = None
        self.depth_writer = None
        self.image_writer = None
        self.match_writer = None

    def setup_subscribers(self):
        """Setup synchronized stereo image subscribers"""
        if ApproximateTimeSynchronizer is None:
            # Fallback to individual subscribers if message_filters not available
            self.get_logger().warn("message_filters not available, using individual subscribers")
            self.left_img_sub = self.create_subscription(
                Image, '/camera/left_image', self.left_image_callback, 10)
            self.right_img_sub = self.create_subscription(
                Image, '/camera/right_image', self.right_image_callback, 10)
        else:
            left_img_sub = Subscriber(self, Image, '/camera/left_image')
            right_img_sub = Subscriber(self, Image, '/camera/right_image')
            
            # Synchronize stereo images
            self.stereo_sync = ApproximateTimeSynchronizer(
                [left_img_sub, right_img_sub], 10, 0.01
            )
            self.stereo_sync.registerCallback(self.stereo_callback)
        
        # Camera info subscriber
        self.left_info_sub = self.create_subscription(
            CameraInfo, '/camera/left_info', self.camera_info_callback, 10
        )
        
        # Initialize image storage for fallback mode
        self.left_image_msg = None
        self.right_image_msg = None

    def setup_feature_detector(self):
        """Initialize ORB feature detector and BruteForce matcher"""
        # ORB parameters
        max_features = self.get_parameter('max_features').value
        scale_factor = self.get_parameter('scale_factor').value
        n_levels = self.get_parameter('n_levels').value
        edge_threshold = self.get_parameter('edge_threshold').value
        first_level = self.get_parameter('first_level').value
        wta_k = self.get_parameter('wta_k').value
        patch_size = self.get_parameter('patch_size').value
        fast_threshold = self.get_parameter('fast_threshold').value
        
        # Create ORB detector
        self.orb = cv2.ORB_create(
            nfeatures=max_features,
            scaleFactor=scale_factor,
            nlevels=n_levels,
            edgeThreshold=edge_threshold,
            firstLevel=first_level,
            WTA_K=wta_k,
            patchSize=patch_size,
            fastThreshold=fast_threshold
        )
        
        # Create BruteForce matcher
        cross_check = self.get_parameter('bf_cross_check').value
        self.bf_matcher = cv2.BFMatcher(cv2.NORM_HAMMING2, crossCheck=cross_check)
        
        # Lowe's ratio test threshold
        self.lowe_ratio = self.get_parameter('lowe_ratio').value

    def setup_stereo_matcher(self):
        """Initialize stereo matcher"""
        num_disp = self.get_parameter('num_disparities').value
        block_size = self.get_parameter('block_size').value
        
        self.stereo_matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=num_disp,
            blockSize=block_size,
            P1=8 * 3 * block_size**2,
            P2=32 * 3 * block_size**2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )

    def camera_info_callback(self, msg):
        """Store camera calibration parameters"""
        if not self.camera_params:
            self.camera_params = {
                'K': np.array(msg.k).reshape(3, 3),
                'D': np.array(msg.d),
                'fx': msg.k[0],
                'fy': msg.k[4],
                'cx': msg.k[2],
                'cy': msg.k[5]
            }
            self.get_logger().info("Camera parameters received")

    def left_image_callback(self, msg):
        """Fallback callback for left image when message_filters not available"""
        self.left_image_msg = msg
        if self.right_image_msg is not None:
            # Check if timestamps are close enough (within 100ms)
            time_diff = abs(msg.header.stamp.sec - self.right_image_msg.header.stamp.sec) + \
                       abs(msg.header.stamp.nanosec - self.right_image_msg.header.stamp.nanosec) * 1e-9
            if time_diff < 0.01:
                self.stereo_callback(self.left_image_msg, self.right_image_msg)

    def right_image_callback(self, msg):
        """Fallback callback for right image when message_filters not available"""
        self.right_image_msg = msg
        if self.left_image_msg is not None:
            # Check if timestamps are close enough (within 100ms)
            time_diff = abs(msg.header.stamp.sec - self.left_image_msg.header.stamp.sec) + \
                       abs(msg.header.stamp.nanosec - self.left_image_msg.header.stamp.nanosec) * 1e-9
            if time_diff < 0.1:
                self.stereo_callback(self.left_image_msg, self.right_image_msg)

    def stereo_callback(self, left_msg, right_msg):
        """Main stereo processing callback"""
        if not self.camera_params:
            return
            
        try:
            with self.lock:
                # Convert images
                left_img = self.bridge.imgmsg_to_cv2(left_msg, "bgr8")
                right_img = self.bridge.imgmsg_to_cv2(right_msg, "bgr8")
                
                # Convert to grayscale
                rgb = self.get_parameter('rgb').value
                if rgb:
                    left_img = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
                    right_img = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
                
                # Compute depth map
                depth_map = self.compute_depth_map(left_img, right_img)
                
                # Process visual odometry
                current_frame = {
                    'image': left_img,
                    'depth': depth_map,
                    'timestamp': left_msg.header.stamp
                }
                
                if self.prev_frame is not None:
                    self.process_visual_odometry(current_frame)
                
                self.prev_frame = current_frame
                self.frame_history.append(current_frame)
                
        except Exception as e:
            self.get_logger().error(f"Error in stereo processing: {str(e)}")

    def compute_depth_map(self, left_gray, right_gray):
        """Compute depth map from stereo pair"""
        # Compute disparity
        disparity = self.stereo_matcher.compute(left_gray, right_gray).astype(np.float32) / 16.0
        
        # Convert disparity to depth
        # Z = (fx * baseline) / disparity
        fx = self.camera_params['fx']
        depth_map = np.zeros_like(disparity)
        
        # Avoid division by zero
        valid_disp = disparity > 0
        depth_map[valid_disp] = (fx * self.baseline) / disparity[valid_disp]

        # ------------------------------------------------------------
        # # Avoid instability and division by zero
        # disparity[disparity == -1.0] = 0.1
        # disparity[disparity == 0.0] = 0.1
        
        # # Make empty depth map then fill with depth
        # depth_map = np.ones(disparity.shape)
        # depth_map = fx * self.baseline / disparity
        # ------------------------------------------------------------
        
        # # Print some statistics about the depth map
        # print('Shape Depth',depth_map.shape)
        # print('Depth at left up corner: ',depth_map[0,0])
        # # print('Depth: ',depth_map[int(479),int(639/2)])
        # print('Max Depth: ',depth_map.max())
        # print('Real Max Depth: ',depth_map[depth_map < depth_map.max()].flatten().max())
        # for i, pixel in enumerate(depth_map[4]):
        #     if pixel < depth_map.max():
        #         print('First non-max value at index', i) # Ans: 96 at sad_window = 6
        #         break

        # Record disparity and depth if recording is enabled
        if self.get_parameter('record_disparity_depth_left_image').value:
            self.record_dis_and_depth(disparity, depth_map, valid_disp, left_gray)

        # Apply depth limits
        max_depth = self.get_parameter('max_depth').value
        min_depth = self.get_parameter('min_depth').value
        depth_map = np.clip(depth_map, min_depth, max_depth)
        
        return depth_map

    def record_dis_and_depth(self, disparity, depth_map, valid_disp, left_gray):
        """ START RECORDING DISPARITY AND DEPTH IMAGES """
        current_time = time.time()
        if self.state == 1:
            self.get_logger().info('Recording finished, not recording anymore.')
            return
        if not self.recording_started:
            self.recording_started = True
            self.record_start_time = current_time
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.disparity_writer = cv2.VideoWriter(os.path.join(self.save_path, 'disparity.mp4'), fourcc, self.fps, self.frame_size)
            self.depth_writer = cv2.VideoWriter(os.path.join(self.save_path, 'depth.mp4'), fourcc, self.fps, self.frame_size)
            self.image_writer = cv2.VideoWriter(os.path.join(self.save_path, 'image.mp4'), fourcc, self.fps, self.frame_size)
            self.get_logger().info('Started recording depth and disparity.')

        if self.recording_started and (current_time - self.record_start_time) < self.recording_duration:
            # Process disparity for display
            disparity /= disparity.max() # normalize the disparity map to the range [0, 1]
            disparity = 1 - disparity # invert the max and min of disparity map (from max = 1, min = 0 to max = 0, min = 1)
            disparity = (disparity * 255).astype('uint8') # convert the disparity map to uint8 format
            disp_colormap = cv2.applyColorMap(disparity, cv2.COLORMAP_RAINBOW)    # apply a color map to the disparity map
                                                                    # COLORMAP_JET,COLORMAP_TURBO,COLORMAP_INFERNO
            # Process depth for display
            depth_normalized = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX)
            depth_uint8 = depth_normalized.astype(np.uint8)
            depth_colormap = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_RAINBOW)

            # Left Gray (convert to BGR for color video output)
            left_resized = cv2.resize(left_gray, self.frame_size)
            left_bgr = cv2.cvtColor(left_resized, cv2.COLOR_GRAY2BGR)

            self.disparity_writer.write(disp_colormap)
            self.depth_writer.write(depth_colormap)
            self.image_writer.write(left_bgr)

        elif self.recording_started and (current_time - self.record_start_time) >= self.recording_duration:
            self.state = 1
            if self.disparity_writer:
                self.disparity_writer.release()
            if self.depth_writer:
                self.depth_writer.release()
            if self.image_writer:
                self.image_writer.release()
            self.recording_started = False
            self.get_logger().info('Finished recording depth and disparity.')

    def process_visual_odometry(self, current_frame):
        """Main visual odometry processing"""
        prev_img = self.prev_frame['image']
        curr_img = current_frame['image']
        depth_map = self.prev_frame['depth']

        # Create mask for valid depth regions
        mask = self.mask_disparity(depth_map) 

        # Extract and match features using ORB and BruteForce
        motion_valid, R, t = self.estimate_motion_orb(prev_img, curr_img, depth_map, mask)
        
        if motion_valid:
            # Create transformation matrix for camera motion
            T_frame = np.eye(4)
            T_frame[:3, :3] = R
            T_frame[:3, 3] = t.ravel()

            # Apply temporal consistency check
            if self.is_motion_consistent(T_frame):
                # Update cumulative transformation
                # For visual odometry: T_world = T_world * T_frame (not inverse)
                self.T_cumulative = self.T_cumulative @ T_frame
                self.publish_odometry(current_frame['timestamp'])
            else:
                self.get_logger().warn("Motion inconsistent, skipping frame")

    def mask_disparity(self, depth):
        mask = np.zeros(depth.shape, dtype=np.uint8)
        ymax = depth.shape[0] # get the maximum height of the depth map
        xmax = depth.shape[1] # get the maximum width of the depth map
        cv2.rectangle(mask, (96,0), (xmax,ymax), (255), thickness = -1)
        # cv2.imshow("mask", mask) # visualize the mask
        # cv2.waitKey(1)
        return mask

    def visualize_matches(self, image1, kp1, image2, kp2, match):
        # self.get_logger().info(f"Visualizing matches")
        """
        Visualize corresponding matches in two images
        """
        if self.state == 1:
            self.get_logger().info('Recording finished, not recording anymore.')
            return

        current_time = time.time()

        if not self.recording_started:
            self.recording_started = True
            self.record_start_time = current_time
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.match_writer = cv2.VideoWriter(os.path.join(self.save_path, 'matches.mp4'), fourcc, self.fps, self.frame_size)
            self.get_logger().info('Started recording depth and disparity.')
        if self.recording_started and (current_time - self.record_start_time) < self.recording_duration:
            image_matches = cv2.drawMatches(image1, kp1, image2, kp2, match, None, flags=2) # draw the matches on the images
            # cv2.imshow("matches", image_matches)
            # cv2.waitKey(1)
            image_matches = cv2.resize(image_matches, self.frame_size)
            self.match_writer.write(image_matches)
        elif self.recording_started and (current_time - self.record_start_time) >= self.recording_duration:
            self.state = 1
            if self.match_writer:
                self.match_writer.release()
                self.match_writer = None
                self.get_logger().info('Finished recording matches.')
            self.recording_started = False

    def estimate_motion_orb(self, prev_img, curr_img, depth_map, mask=None):
        """Estimate camera motion between frames using ORB features and BruteForce matching"""
        # Detect ORB keypoints and descriptors in both images
        kp1, des1 = self.orb.detectAndCompute(prev_img, mask)
        kp2, des2 = self.orb.detectAndCompute(curr_img, mask)
        
        if des1 is None or des2 is None or len(kp1) < 50 or len(kp2) < 50:
            return False, None, None
        
        # Match features using BruteForce matcher
        if self.get_parameter('bf_cross_check').value:
            # Use cross-check matching (more reliable but fewer matches)
            matches = self.bf_matcher.match(des1, des2)
            # Sort matches by distance
            matches = sorted(matches, key=lambda x: x.distance)
            # print(f"Found {len(matches)} matches with cross-check")
        else:
            # Use k-nearest neighbors matching with Lowe's ratio test
            matches = self.bf_matcher.knnMatch(des1, des2, k=2)
            
            # Apply Lowe's ratio test
            good_matches = []
            for match_pair in matches:
                if len(match_pair) == 2:
                    m, n = match_pair
                    if m.distance < self.lowe_ratio * n.distance:
                        good_matches.append(m)
            matches = good_matches
            # print(f"Found {len(matches)} good matches after Lowe's ratio test")
        
        if len(matches) < self.get_parameter('min_matches').value:
            self.get_logger().warn(f"Not enough matches found: {len(matches)} < {self.get_parameter('min_matches').value}")
            return False, None, None
        
        # Visualize matches if enabled
        if self.get_parameter('record_matches').value:
            self.visualize_matches(prev_img, kp1, curr_img, kp2, matches)

        # Extract matched keypoints
        prev_pts = np.array([kp1[m.queryIdx].pt for m in matches], dtype=np.float32)
        curr_pts = np.array([kp2[m.trainIdx].pt for m in matches], dtype=np.float32)
        
        # Get 3D points from depth map for previous frame keypoints
        object_points = self.get_3d_points(prev_pts, depth_map)
        image_points = curr_pts
        
        # Filter points with valid depth
        valid_mask = ~np.isnan(object_points).any(axis=1)
        if np.sum(valid_mask) < 20:
            return False, None, None
            
        object_points = object_points[valid_mask]
        image_points = image_points[valid_mask]
        
        # Solve PnP with RANSAC
        try:
            # success, rvec, tvec, inliers = cv2.solvePnPRansac(
            #     object_points, image_points, self.camera_params['K'], 
            #     self.camera_params['D'], reprojectionError=self.get_parameter('ransac_threshold').value
            # )
            success, rvec, tvec, inliers = cv2.solvePnPRansac(
                object_points, image_points, self.camera_params['K'], None
            )
            
            if success and len(inliers) > 15:
                R, _ = cv2.Rodrigues(rvec)
                return True, R, tvec
                
        except cv2.error as e:
            self.get_logger().warn(f"PnP failed: {str(e)}")
        
        return False, None, None

    def get_3d_points(self, pixel_coords, depth_map):
        """Convert pixel coordinates to 3D points using depth"""
        fx, fy = self.camera_params['fx'], self.camera_params['fy']
        cx, cy = self.camera_params['cx'], self.camera_params['cy']
        
        points_3d = []
        for pt in pixel_coords:
            u, v = int(pt[0]), int(pt[1])
            if 0 <= u < depth_map.shape[1] and 0 <= v < depth_map.shape[0]:
                depth = depth_map[v, u]
                if depth > 0:
                    x = (u - cx) * depth / fx
                    y = (v - cy) * depth / fy
                    points_3d.append([x, y, depth])
                else:
                    points_3d.append([np.nan, np.nan, np.nan])
            else:
                points_3d.append([np.nan, np.nan, np.nan])
        
        return np.array(points_3d, dtype=np.float32)

    def is_motion_consistent(self, T_frame):
        """Check if motion is consistent with recent history"""
        if len(self.frame_history) < 2:
            return True
            
        # Extract translation magnitude
        translation_norm = np.linalg.norm(T_frame[:3, 3])
        
        # Check for reasonable motion (not too large jumps)
        max_translation = self.get_parameter('max_translation').value
        if translation_norm > max_translation:  # 2 meters per frame is unrealistic
            self.get_logger().warn(f"Large translation detected: {translation_norm:.2f} m")
            return False
            
        return True

    def publish_odometry(self, timestamp):
        """Publish odometry message"""
        # The T_cumulative represents camera pose in world frame
        # For robotics, we need base_link pose, so we need camera-to-base transform
        # Camera to base_link transformation (adjust these values based on your robot setup)
        # Assuming camera is mounted facing forward on the robot
        T_cam_to_base = np.eye(4)
        # If camera is at some offset from base_link, adjust these:
        T_cam_to_base[:3, 3] = [0.0, 0.0, 0.0]  # [x_offset, y_offset, z_offset]
        
        # Robot pose in world frame = camera pose * cam_to_base transform
        T_robot = self.T_cumulative @ T_cam_to_base
        
        # Extract position and orientation
        position = T_robot[:3, 3]
        
        # Fix coordinate system: camera frame to robot frame
        # Standard robot convention: x=forward, y=left, z=up
        # Camera convention: z=forward, x=right, y=down
        # Apply coordinate transformation
        position_robot = np.array([
            -position[2],  # camera z (forward) -> robot x (forward), but negated because VO gives relative motion
            -position[0],  # camera x (right) -> robot y (left), FIXED: negated to correct left/right reversal
            -position[1]   # camera y (down) -> robot z (up), negated
        ])
        
        # For rotation, we need to convert from camera frame to robot frame
        R_cam = T_robot[:3, :3]
        
        # Coordinate frame conversion matrix (camera to robot)
        # FIXED: Updated rotation matrix to match the corrected position transformation
        R_cam_to_robot = np.array([
            [0, 0, -1],  # camera z -> -robot x
            [-1, 0, 0],  # camera x -> -robot y (FIXED: negated to match position fix)
            [0, -1, 0]   # camera y -> -robot z
        ])
        
        # Apply coordinate transformation
        R_robot = R_cam_to_robot @ R_cam @ R_cam_to_robot.T
        
        # Create the current robot pose matrix
        T_current = np.eye(4)
        T_current[:3, :3] = R_robot
        T_current[:3, 3] = position_robot
        
        # Apply robot frame offset
        offset_x = self.get_parameter('offset_x').value
        offset_y = self.get_parameter('offset_y').value
        offset_z = self.get_parameter('offset_z').value
        offset_yaw = self.get_parameter('offset_yaw').value
        
        # Create offset transformation matrix
        T_offset = np.eye(4)
        # Translation offset
        T_offset[:3, 3] = [offset_x, offset_y, offset_z]
        # Rotation offset (yaw rotation around z-axis)
        cos_yaw = np.cos(offset_yaw)
        sin_yaw = np.sin(offset_yaw)
        T_offset[:3, :3] = np.array([
            [cos_yaw, -sin_yaw, 0],
            [sin_yaw,  cos_yaw, 0],
            [0,        0,       1]
        ])
        
        # Apply offset: T_final = T_offset * T_current
        T_final = T_offset @ T_current
        
        # Extract final position and orientation
        position_final = T_final[:3, 3]
        R_final = T_final[:3, :3]
        
        # Update final transformation matrix for quaternion conversion
        T_for_quat = np.eye(4)
        T_for_quat[:3, :3] = R_final
        T_for_quat[:3, 3] = position_final
        
        # Convert to quaternion
        quaternion = quaternion_from_matrix(T_for_quat)
        
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        # Position (using offset position)
        odom_msg.pose.pose.position.x = float(position_final[0])
        odom_msg.pose.pose.position.y = float(position_final[1])
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation
        odom_msg.pose.pose.orientation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )
        
        # Add covariance (simple diagonal)
        # odom_msg.pose.covariance[0] = 0.1   # x
        # odom_msg.pose.covariance[7] = 0.1   # y
        # odom_msg.pose.covariance[14] = 0.1  # z
        # odom_msg.pose.covariance[21] = 0.1  # roll
        # odom_msg.pose.covariance[28] = 0.1  # pitch
        # odom_msg.pose.covariance[35] = 0.1  # yaw
        
        self.odom_pub.publish(odom_msg)
        
        # Publish TF if needed
        if self.get_parameter('publish_tf').value:
            self.publish_tf(timestamp, position_final, quaternion)

    def publish_tf(self, timestamp, position, quaternion):
        """Publish TF transform"""
        tf_msg = TransformStamped()
        tf_msg.header.stamp = timestamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        
        tf_msg.transform.translation.x = float(position[0])
        tf_msg.transform.translation.y = float(position[1])
        tf_msg.transform.translation.z = float(position[2])
        
        tf_msg.transform.rotation = Quaternion(
            x=quaternion[0], y=quaternion[1],
            z=quaternion[2], w=quaternion[3]
        )
        
        self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = OptimizedStereoVO()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()