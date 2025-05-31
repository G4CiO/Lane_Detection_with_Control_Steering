#!/usr/bin/python3

from rover_controller.dummy_module import dummy_function, dummy_var
import rclpy
import os
import yaml
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ament_index_python import get_package_share_directory
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from datetime import datetime

class ControllerServer(Node):
    def __init__(self):
        super().__init__('controller_server_node')
        self.get_logger().info('controller_server_node has been start')

        # Declare parameters
        self.declare_parameter('save_data', False)  # Parameter to control data saving
        self.declare_parameter('file', 'path.yaml')
        self.declare_parameter('filename', 'odometry_data')
        self.declare_parameter('control_mode', 'stanley')  # Default mode: 'pure_pursuit'
        self.declare_parameter('path_file', 'path.yaml')
        self.declare_parameter('wheelbase', 0.2) 
        self.declare_parameter('kp', 1.0)  # Set k parameter dynamically
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('wheelradius', 0.045)   # meters
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        # Declare the odometry source parameter, default to "ground_truth" 
        self.declare_parameter('odom_source', 'ground_truth')
        
        # self.gt_data = []  # ground truth data
        # self.vo_data = []  # visual odometry data
        # Initialize data storage
        self.ground_truth_data = []
        self.visual_odom_data = []
        self.synchronized_data = []
        self.latest_ground_truth = None
        self.count = 0
        
    
        # Subscribe to visual odometry
        if self.get_parameter('save_data').value:
            self.create_subscription(Odometry, '/odometry/visual_odom', self.visual_odom_callback, 10)
        # Subscribe to ground truth odometry
        self.odom_sub = self.create_subscription(Odometry, '/odometry/ground_truth', self.odom_callback, 10)

        # Initialization
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.v_avr = 0.0
        self.state = 0

        pkg_name = 'rover_controller'
        path_pkg_share_path = get_package_share_directory(pkg_name)
        ws_path, _ = path_pkg_share_path.split('install')
        file = self.get_parameter('file').value
        self.path_path = os.path.join(ws_path, 'src', pkg_name, 'config', file)
        filename = self.get_parameter('filename').value
        filename_yaml = f'{filename}.yaml'
        self.file_path = os.path.join(ws_path, 'src', pkg_name, 'data', filename_yaml)
        self.get_logger().info(f'Path to save data: {self.file_path}')
        

        # Stanley controllers
        self.linear_velo_stan = 0.5
        self.k = 5.0

        # Load path from YAML file
        self.path = self.load_path()
        self.current_target_idx = 71
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(JointState, '/joint_states', self.jointstates_callback, 10)

        # Timer for control loop
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.timer_callback)  # 100 Hz

    def jointstates_callback(self, msg: JointState):
        """ Callback to get JointState from /joint_states topic """
        wheelradius = self.get_parameter('wheelradius').value
        index_rl, index_rr = None, None
        
        for i in range(len(msg.name)):
            if msg.name[i] == "rear_left_wheel":
                index_rl = i
            elif msg.name[i] == "rear_right_wheel":
                index_rr = i

        if index_rl is not None and index_rr is not None:
            v_rl = msg.velocity[index_rl] * wheelradius
            v_rr = msg.velocity[index_rr] * wheelradius
            self.v_avr = (v_rl + v_rr) / 2

    def load_path(self):
        with open(self.path_path, 'r') as file:
            return yaml.safe_load(file)
    
    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        self.robot_yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]
        # self.gt_data.append({'x': self.robot_x, 'y': self.robot_y})
        """Store the latest ground truth odometry data"""
        self.latest_ground_truth = {
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y
        }

    def visual_odom_callback(self, msg: Odometry):
        # x = msg.pose.pose.position.x
        # y = msg.pose.pose.position.y
        # self.gt_data.append({'x': self.robot_x, 'y': self.robot_y})
        # self.vo_data.append({'x': x, 'y': y})
        """Process visual odometry and synchronize with ground truth"""
        visual_data = {
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y
        }
        # Synchronize with latest ground truth data
        if self.latest_ground_truth is not None:
            synchronized_point = {
                'ground_truth': {
                    'x': self.latest_ground_truth['x'],
                    'y': self.latest_ground_truth['y'],
                    'timestamp': self.latest_ground_truth['timestamp']
                },
                'visual_odom': {
                    'x': visual_data['x'],
                    'y': visual_data['y'],
                    'timestamp': visual_data['timestamp']
                }
            }
            
            self.synchronized_data.append(synchronized_point)
            
            # Save data to file
            # self.save_data()
            
            # self.get_logger().info(
            #     f'Data point {len(self.synchronized_data)}: '
            #     f'GT({self.latest_ground_truth["x"]:.3f}, {self.latest_ground_truth["y"]:.3f}) '
            #     f'VO({visual_data["x"]:.3f}, {visual_data["y"]:.3f})'
            # )

    def save_data(self):
        """Save synchronized data to YAML file"""
        try:
            data_to_save = {
                'metadata': {
                    'total_points': len(self.synchronized_data),
                    'created_at': datetime.now().isoformat(),
                    'description': 'Synchronized odometry data from ground truth and visual odometry'
                },
                'synchronized_data': self.synchronized_data
            }
            
            with open(self.file_path, 'w') as file:
                yaml.dump(data_to_save, file, default_flow_style=False)
                
        except Exception as e:
            self.get_logger().error(f'Error saving data: {str(e)}')

    def pub_cmd(self, vx, wz):
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.cmd_vel_pub.publish(msg)

    def normalize_angle(self, angle):
        # Normalize an angle to [-pi, pi]
        normalize_angle = math.atan2(math.sin(angle), math.cos(angle))
        return normalize_angle

    def serch_nearest_point_index(self):
        # Search nearest point index
        if self.state == 0:
            dx = [self.robot_x - target_x['x'] for target_x in self.path]
            dy = [self.robot_y - target_y['y'] for target_y in self.path]
            d = np.hypot(dx, dy)
            self.current_target_idx = np.argmin(d)
            self.state = 1

    def timer_callback(self):
        control_mode = self.get_parameter('control_mode').value
        if control_mode == 'pid':
            self.pid_control()
        elif control_mode == 'pure_pursuit':
            self.pure_pursuit_control()
        elif control_mode == 'stanley':
            self.stanley_control()
        else:
            self.get_logger().warn(f"Unknown mode '{control_mode}', defaulting to Pure Pursuit control")
            self.pure_pursuit_control()

    def stanley_control(self):
        wheelbase = self.get_parameter('wheelbase').value
        # if self.current_target_idx >= len(self.path) - 1:
        #     self.pub_cmd(0.0, 0.0)
        #     return # Stop
        save_data = self.get_parameter('save_data').value
        if self.current_target_idx == 70:
            if save_data and self.count == 0:
                self.save_data()
                self.get_logger().info("Path completed, odometry data saved.")
            self.pub_cmd(0.0, 0.0)
            self.count = 1
            return # Stop

        # Calc front axle position
        fx = self.robot_x + (wheelbase/2 * np.cos(self.robot_yaw))
        fy = self.robot_y + (wheelbase/2 * np.sin(self.robot_yaw))

        # Search nearest point index
        dx = [fx - target_x['x'] for target_x in self.path]
        dy = [fy - target_y['y'] for target_y in self.path]
        d = np.hypot(dx, dy)
        self.current_target_idx = np.argmin(d)
        target = self.path[self.current_target_idx]

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(self.robot_yaw + np.pi / 2),-np.sin(self.robot_yaw + np.pi / 2)]
        e_fa = np.dot([dx[self.current_target_idx], dy[self.current_target_idx]], front_axle_vec)
  
        # Compute heading error
        theta_e = target['yaw'] - self.robot_yaw
        # Normalize an angle to [-pi, pi]
        theta_e = self.normalize_angle(theta_e)

        # Stanley control formula
        if self.v_avr != 0.0:
            delta = theta_e + np.arctan2(self.k * e_fa, self.v_avr)
            delta = max(-0.6, min(delta, 0.6))
        else:
            delta = 0.0

        # Angular Velocity Calculation (ω)
        angular_velocity = (self.linear_velo_stan * math.tan(delta)) / wheelbase

        # Publish cmd_vel
        self.pub_cmd(self.linear_velo_stan, angular_velocity)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish zero cmd_vel before shutting down
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        node.cmd_vel_pub.publish(cmd)
        node.get_logger().info("Published zero cmd_vel before shutdown.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()