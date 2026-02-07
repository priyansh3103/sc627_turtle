#!/usr/bin/env python3
"""
Map Publisher Node with C-Space Inflation

This node publishes:
- /map: Original occupancy grid from YAML/PGM file
- /map_inflated: Inflated occupancy grid using Minkowski Sum concept

The inflation expands obstacles by (robot_radius + safety_margin) to create
the Configuration Space (C-Space) representation.
"""

import os
import yaml
import numpy as np
from PIL import Image
from scipy import ndimage

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import Pose


class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        
        # Declare parameters
        self.declare_parameter('map_yaml_file', '')
        self.declare_parameter('robot_radius', 0.105)  # TurtleBot3 Burger radius ~0.105m
        self.declare_parameter('safety_margin', 0.05)  # Additional safety margin
        self.declare_parameter('publish_rate', 1.0)    # Hz
        self.declare_parameter('frame_id', 'map')
        
        # Get parameters
        self.map_yaml_file = self.get_parameter('map_yaml_file').get_parameter_value().string_value
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value
        self.safety_margin = self.get_parameter('safety_margin').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        if not self.map_yaml_file:
            self.get_logger().error('No map_yaml_file parameter provided!')
            return
            
        self.get_logger().info(f'Loading map from: {self.map_yaml_file}')
        self.get_logger().info(f'Robot radius: {self.robot_radius}m')
        self.get_logger().info(f'Safety margin: {self.safety_margin}m')
        self.get_logger().info(f'Total inflation: {self.robot_radius + self.safety_margin}m')
        
        # QoS profile for map (latched/transient local)
        map_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/map_stored', map_qos)
        self.map_inflated_pub = self.create_publisher(OccupancyGrid, '/map_inflated', map_qos)
        
        # Load and process the map
        self.load_map()
        
        # Create timer for periodic publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_maps)
        
        # Publish immediately
        self.publish_maps()
        
    def load_map(self):
        """Load map from YAML file and corresponding image."""
        try:
            with open(self.map_yaml_file, 'r') as f:
                map_data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load YAML file: {e}')
            return
            
        # Get map directory for relative image path
        map_dir = os.path.dirname(self.map_yaml_file)
        
        # Parse YAML data
        image_file = map_data.get('image', '')
        if not os.path.isabs(image_file):
            image_file = os.path.join(map_dir, image_file)
            
        self.resolution = map_data.get('resolution', 0.05)
        self.origin = map_data.get('origin', [0.0, 0.0, 0.0])
        self.negate = map_data.get('negate', 0)
        self.occupied_thresh = map_data.get('occupied_thresh', 0.65)
        self.free_thresh = map_data.get('free_thresh', 0.25)
        self.mode = map_data.get('mode', 'trinary')
        
        self.get_logger().info(f'Loading image: {image_file}')
        self.get_logger().info(f'Resolution: {self.resolution}m/pixel')
        self.get_logger().info(f'Origin: {self.origin}')
        
        # Load image
        try:
            img = Image.open(image_file)
            img_array = np.array(img)
        except Exception as e:
            self.get_logger().error(f'Failed to load image file: {e}')
            return
            
        # Convert to occupancy grid values
        # PGM: 0=black(occupied), 255=white(free)
        # OccupancyGrid: 0=free, 100=occupied, -1=unknown
        
        self.height, self.width = img_array.shape[:2]
        
        # Normalize to 0-1 range
        if img_array.max() > 0:
            normalized = img_array.astype(float) / 255.0
        else:
            normalized = img_array.astype(float)
            
        # Apply negate if specified
        if self.negate:
            normalized = 1.0 - normalized
        else:
            # Standard PGM: white=free, black=occupied, so invert
            normalized = 1.0 - normalized
            
        # Convert to occupancy grid format
        self.occupancy_data = np.zeros(img_array.shape[:2], dtype=np.int8)
        
        for i in range(self.height):
            for j in range(self.width):
                val = normalized[i, j]
                if val >= self.occupied_thresh:
                    self.occupancy_data[i, j] = 100  # Occupied
                elif val <= self.free_thresh:
                    self.occupancy_data[i, j] = 0    # Free
                else:
                    self.occupancy_data[i, j] = -1   # Unknown
                    
        self.get_logger().info(f'Map loaded: {self.width}x{self.height} pixels')
        
        # Create inflated map using Minkowski Sum
        self.create_inflated_map()
        
    def create_inflated_map(self):
        """
        Create inflated map using Minkowski Sum concept.
        
        The Minkowski Sum inflates obstacles by the robot radius + safety margin.
        This is implemented using morphological dilation with a circular 
        structuring element.
        
        Output values:
        - 100: Original obstacles
        - 99: Inflated/C-Space boundary (not original obstacle)
        - 0: Free space
        - -1: Unknown
        """
        inflation_distance = self.robot_radius + self.safety_margin
        inflation_cells = int(np.ceil(inflation_distance / self.resolution))
        
        self.get_logger().info(f'Inflating obstacles by {inflation_distance}m ({inflation_cells} cells)')
        
        # Create circular structuring element (disk) for Minkowski Sum
        # This represents the robot footprint
        y, x = np.ogrid[-inflation_cells:inflation_cells+1, -inflation_cells:inflation_cells+1]
        structuring_element = x*x + y*y <= inflation_cells*inflation_cells
        
        # Create binary obstacle map (1 = occupied, 0 = not occupied)
        obstacle_mask = (self.occupancy_data == 100).astype(np.uint8)
        
        # Apply morphological dilation (Minkowski Sum with disk)
        inflated_obstacles = ndimage.binary_dilation(
            obstacle_mask, 
            structure=structuring_element
        ).astype(np.uint8)
        
        # Create inflated occupancy grid with differentiated values
        # 100 = original obstacle, 99 = inflated boundary
        self.inflated_data = self.occupancy_data.copy()
        
        for i in range(self.height):
            for j in range(self.width):
                if inflated_obstacles[i, j] == 1:
                    if self.occupancy_data[i, j] == 100:
                        # Original obstacle stays at 100
                        self.inflated_data[i, j] = 100
                    elif self.occupancy_data[i, j] != -1:
                        # Inflated region (not original obstacle, not unknown) = 99
                        self.inflated_data[i, j] = 99
                    # Unknown cells (-1) that get inflated also become 99
                    elif self.occupancy_data[i, j] == -1:
                        self.inflated_data[i, j] = 99
                    
        # Count statistics
        original_occupied = np.sum(self.occupancy_data == 100)
        inflated_boundary = np.sum(self.inflated_data == 99)
        total_occupied = np.sum(self.inflated_data >= 99)
        
        self.get_logger().info(f'Original obstacle cells (100): {original_occupied}')
        self.get_logger().info(f'Inflated boundary cells (99): {inflated_boundary}')
        self.get_logger().info(f'Total occupied cells (>=99): {total_occupied}')
        
    def create_occupancy_grid_msg(self, data, stamp):
        """Create OccupancyGrid message from numpy array."""
        msg = OccupancyGrid()
        
        # Header
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        
        # Map metadata
        msg.info = MapMetaData()
        msg.info.map_load_time = stamp
        msg.info.resolution = float(self.resolution)
        msg.info.width = self.width
        msg.info.height = self.height
        
        # Origin pose
        msg.info.origin = Pose()
        msg.info.origin.position.x = float(self.origin[0])
        msg.info.origin.position.y = float(self.origin[1])
        msg.info.origin.position.z = 0.0
        
        # Handle yaw in origin (third element)
        if len(self.origin) > 2:
            yaw = float(self.origin[2])
            msg.info.origin.orientation.z = np.sin(yaw / 2.0)
            msg.info.origin.orientation.w = np.cos(yaw / 2.0)
        else:
            msg.info.origin.orientation.w = 1.0
            
        # Flatten data in row-major order (ROS expects row-major)
        # Flip vertically because image origin is top-left, map origin is bottom-left
        flipped_data = np.flipud(data)
        msg.data = flipped_data.flatten().tolist()
        
        return msg
        
    def publish_maps(self):
        """Publish both original and inflated maps."""
        if not hasattr(self, 'occupancy_data'):
            return
            
        stamp = self.get_clock().now().to_msg()
        
        # Publish original map
        map_msg = self.create_occupancy_grid_msg(self.occupancy_data, stamp)
        self.map_pub.publish(map_msg)
        
        # Publish inflated map
        inflated_msg = self.create_occupancy_grid_msg(self.inflated_data, stamp)
        self.map_inflated_pub.publish(inflated_msg)
        
        self.get_logger().debug('Published /map and /map_inflated')


def main(args=None):
    rclpy.init(args=args)
    
    node = MapPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
