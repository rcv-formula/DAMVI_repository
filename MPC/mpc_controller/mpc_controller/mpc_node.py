import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
import numpy as np
import yaml
import os
import cvxpy as cp
from tf_transformations import euler_from_quaternion
from ament_index_python.packages import get_package_share_directory
import time
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped

class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        
        # Load parameters from YAML file
        self.load_config()
        self.global_path = None

        # State and control variables
        self.state = np.zeros(4)  # [x, y, theta, v]
        self.control = [0.0, 0.0]  # [acceleration, steering_rate]
        self.cost_map = None  # Placeholder for the 2.5D cost map
        
        # Subscriptions
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, '/local_costmap', self.cost_map_callback, 10)
        self.create_subscription(Path, '/global_path', self.global_path_callback, 10)
        
        # Publisher
        self.local_path_publisher = self.create_publisher(Path, '/local_path', 10)
        self.cmd_drive_publisher = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)
        
        # Timer for MPC loop
        self.create_timer(self.dt, self.run_mpc)
        
        self.get_logger().info("MPC Controller Initialized")

    def load_config(self):
        """Load parameters from a YAML file."""
        package_dir = get_package_share_directory('mpc_controller')  # Replace with your package name
        config_path = os.path.join(package_dir, 'config', 'config.yaml')
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
            self.validate_config(config) # check if config file is ok
        
        # Load car specifications
        car_config = config['car']
        self.wheel_base = car_config['wheel_base']
        self.wheel_radius = car_config['wheel_radius']
        self.track_width = car_config['track_width']
        self.max_speed = car_config['max_speed']
        self.max_steering_angle = car_config['max_steering_angle']
        
        # Load MPC parameters
        mpc_config = config['mpc']
        self.horizon = mpc_config['horizon']
        self.dt = mpc_config['dt']
        self.max_acceleration = mpc_config['max_acceleration']
        self.max_steering_rate = mpc_config['max_steering_rate']
        
        # Load MPC weights
        weights_config = config['weights']
        self.Q = np.diag(weights_config['Q'])
        self.Qf = np.diag(weights_config['Qf'])
        self.R = np.diag(weights_config['R'])

        # Load Map Resolution
        map_config = config['map']
        self.out_of_bounds_penalty = map_config['out_of_bounds_penalty']
        
        self.get_logger().info(f"Loaded configuration from {config_path}")

    def validate_config(self, config):
        required_keys = ['car', 'mpc', 'weights', 'map']
        for key in required_keys:
            if key not in config:
                raise ValueError(f"Missing required key in config: {key}")
            # Additional key-specific validation can be added here

    def odom_callback(self, msg):
        """Updates the robot's state from odometry."""

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        
        self.state[0] = msg.pose.pose.position.x
        self.state[1] = msg.pose.pose.position.y
        self.state[2] = yaw
        self.state[3] = msg.twist.twist.linear.x

    def cost_map_callback(self, msg):
        """Updates the cost map."""
        # Assuming a flattened 2D grid with height values in meters.
        # Extract Map Data
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = (msg.info.origin.position.x, msg.info.origin.position.y) #Bottom-left corner of the map

        # Convert the occupancy data to a 2D numpy array
        map_data = np.array(msg.data).reshape((height, width))

        # Normalize map values (convert -1 to high penalty, 0 to free, and >0 to obstacle cost)
        self.cost_map = np.where(map_data == -1, self.out_of_bounds_penalty, map_data)

        # Save map metadata
        self.map_resolution = resolution
        self.map_origin = origin

        self.get_logger().debug(f"Cost map updated: size={width}x{height}, resolution={resolution}, origin={origin}")


    def global_path_callback(self, msg):
        """Updates the global path."""
        self.global_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.get_logger().debug(f"Received global path with {len(self.global_path)} waypoints")

    def find_closest_point(self):
        """Finds the closest point on the global path to the current state."""
        if self.global_path is None or len(self.global_path) == 0:
            self.get_logger().warn("Global path is unavailable.")
            return None
    
        current_position = np.array([self.state[0], self.state[1]])
        distances = [np.linalg.norm(current_position - np.array(point)) for point in self.global_path]
        return np.argmin(distances)  # Index of the closest point

    def generate_local_trajectory(self, closest_idx):
        """Generates a local trajectory from the global path."""
        if self.global_path is None or len(self.global_path) == 0:
            return None
        
        end_idx = min(closest_idx + self.horizon + 1, len(self.global_path))
        local_path = self.global_path[closest_idx:end_idx]
        
        # Pad with the last point if the global path is shorter than the horizon 
        if len(local_path) < self.horizon + 1:
            self.get_logger().warn("Global path too short for the horizon. Padding local path.")
            while len(local_path) < self.horizon + 1:
                local_path.append(local_path[-1])  # Repeat the last point
        
        return local_path
    
    def publish_local_path(self, local_path):
        """Publishes the local trajectory as a nav_msgs/Path."""
        path_msg = Path()
        path_msg.header.frame_id = "base_link"  # Update to match your map frame
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for point in local_path:
            pose = PoseStamped()
            pose.header.frame_id = "base_link"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.w = 1.0  # Neutral orientation
            path_msg.poses.append(pose)
        
        self.local_path_publisher.publish(path_msg)
        self.get_logger().debug(f"Published local path with {len(local_path)} points.")
    
    def map_cost(self, x, y):
        """Fetch cost from the cost map based on x, y."""
        if self.cost_map is None:
            self.get_logger().warn("Cost map is unavailable. Returning zero cost.")
            return 0  # No penalty if the cost map is unavailable
        
        # Convert world coordinates to grid indices
        grid_x = int((x - self.map_origin[0]) / self.map_resolution)
        grid_y = int((y - self.map_origin[1]) / self.map_resolution)
        
        # Check bounds
        if 0 <= grid_x < self.cost_map.shape[1] and 0 <= grid_y < self.cost_map.shape[0]:
            return self.cost_map[grid_y, grid_x]
        else:
            self.get_logger().debug(f"Position ({x:.2f}, {y:.2f}) is outside cost map bounds.")
            return self.out_of_bounds_penalty  # Penalize positions outside the map

    def run_mpc(self):

        if self.cost_map is None:
            self.get_logger().warn("Skipping MPC as cost map is unavailable.")
            return

        if self.global_path is None or len(self.global_path) == 0:
            self.get_logger().warn("Global path is unavailable. Skipping MPC computation.")
            return
        
        start_time = time.time()

        # State and control optimization variables
        x = cp.Variable((self.horizon + 1, 4))  # State variables
        u = cp.Variable((self.horizon, 2))  # Control variables
        

        # Find the closest point on the global path
        closest_idx = self.find_closest_point()
        if closest_idx is None:
            self.get_logger().warn("No valid global path. Skipping MPC computation.")
            return

        # Generate a local trajectory from the global path
        local_path = self.generate_local_trajectory(closest_idx)
        if local_path is None:
            self.get_logger().warn("Failed to generate local trajectory. Skipping MPC computation.")
            return

        # Publish the local path for visualization
        self.publish_local_path(local_path)

        # Constraints and cost
        constraints = []
        cost = 0.0

        # Initial state constraint
        constraints.append(x[0] == self.state)

        # Dynamics and cost accumulation
        for k in range(self.horizon):
            # Dynamics (simple bicycle model = ackermann steering model)
            # Estimation of the Future by using Dynamics
            constraints.extend([
                x[k + 1, 0] == x[k, 0] + self.dt * x[k, 3] * cp.cos(x[k, 2]),
                x[k + 1, 1] == x[k, 1] + self.dt * x[k, 3] * cp.sin(x[k, 2]),
                x[k + 1, 2] == x[k, 2] + self.dt * u[k, 1],
                x[k + 1, 3] == x[k, 3] + self.dt * u[k, 0],
            ])

            # State constraints
            constraints.append(x[k + 1, 3] >= 0)  # No negative velocity

            # Control constraints
            constraints.extend([
                cp.abs(u[k, 0]) <= self.max_acceleration,  # Acceleration limits
                cp.abs(u[k, 1]) <= self.max_steering_rate,  # Steering rate limits
            ])

            # Path-following cost
            target_state = np.array([local_path[k][0], local_path[k][1], 0.0, 0.0])  # [x, y, theta, v]
            cost += cp.quad_form(x[k, :2] - target_state[:2], self.Q[:2, :2])  # Deviation from local trajectory

            # Control effort cost
            cost += cp.quad_form(u[k], self.R)

            # Obstacle avoidance cost
            cost += self.map_cost(x[k, 0], x[k, 1])  # Penalty from cost map

        # Terminal cost for the last point in the local trajectory
        terminal_state = np.array([local_path[-1][0], local_path[-1][1], self.state[2], self.state[3]]) # [x, y, theta, v]
        cost += cp.quad_form(x[self.horizon] - terminal_state, self.Qf)
        
        # Solve the optimization
        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.OSQP)
        
        if prob.status == cp.OPTIMAL:
            # Apply the first control input
            self.control = u.value[0]
            self.publish_cmd_drive()
        else:
            self.get_logger().warn("MPC optimization failed, resetting")
            self.control = [0.0, 0.0]  # Reset to zero if undefined
        
        # for debugging, the frequency of the control loop (1/self.dt) should match the MPC computation time.
        loop_time = time.time() - start_time
        if loop_time > self.dt:
            self.get_logger().warn(f"MPC loop exceeds real-time requirements: {loop_time:.4f}s > {self.dt}s")
        self.get_logger().debug(f"MPC computation time: {loop_time:.4f}s")
        self.get_logger().debug(f"Solver status: {prob.status}")
        self.get_logger().debug(f"Solver iterations: {prob.solver_stats.num_iters}")
        self.get_logger().debug(f"Solver time: {prob.solver_stats.solve_time:.4f}s")
        self.get_logger().debug(f"Final cost: {prob.value:.4f}")

    def publish_cmd_drive(self):
        """Publishes the control commands as AckermannDriveStamped."""
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"  # Frame of reference

        speed = max(min(self.control[0], self.max_speed), -self.max_speed)  # Clip speed
        steering_angle = max(min(self.control[1], self.max_steering_angle), -self.max_steering_angle)  # Clip angle
        msg.drive.speed = speed
        msg.drive.steering_angle = steering_angle

        self.cmd_drive_publisher.publish(msg)
        self.get_logger().debug(f"Published drive command: speed={self.control[0]:.2f}, angle={self.control[1]:.2f}")

    def destroy_node(self):
        self.get_logger().info("Shutting down MPC Controller.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MPCController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

