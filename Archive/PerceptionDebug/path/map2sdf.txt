import rclpy
import numpy as np
from rclpy.node import Node

# GridMap is used to receive the incoming 2.5D elevation data from the robot's perception system.
from grid_map_msgs.msg import GridMap

# PointCloud2 is used to publish our debug heatmaps and 3D points. 
# It's much lighter and less buggy for RViz to render than GridMap.
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

from message_filters import ApproximateTimeSynchronizer, Subscriber

# We use scipy's Euclidean Distance Transform (EDT) to efficiently calculate 
# the distance from every grid cell to the nearest wall.
from scipy.ndimage import distance_transform_edt

class PostureOptimizerDebug(Node):
    """
    This node acts as a geometric feasibility tester for a quadruped robot.
    It takes in floor and ceiling elevation maps and calculates the safest 
    physical posture (body height and leg span) for every single grid coordinate.
    
    The algorithm is a Brute-Force Max-Min Bottleneck Optimizer:
    For every coordinate, it tests 15 different heights, calculates the required
    leg width for each height, and scores them based on how much "buffer space"
    they leave between the robot and the walls/ceiling.
    """
    def __init__(self):
        super().__init__('posture_optimizer_debug')

        # 1. Subscriptions & Synchronization
        # We need both the floor and ceiling to compute available space. 
        # ApproximateTimeSynchronizer ensures we only process pairs of maps that were 
        # captured at roughly the same time (within 0.5 seconds).
        self.sub_floor = Subscriber(self, GridMap, '/elevation_map')
        self.sub_ceil  = Subscriber(self, GridMap, '/ceiling_elevation_map_flipped')
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_floor, self.sub_ceil], queue_size=10, slop=0.5)
        self.sync.registerCallback(self.callback)

        # 2. PointCloud2 Publishers
        # Using point clouds allows us to easily color-code 2D heatmaps (using the 'intensity' field)
        # or place specific 3D markers without confusing RViz's elevation rendering.
        self.pub_h_avail = self.create_publisher(PointCloud2, '/debug_h_avail_pc', 10)
        self.pub_w_avail = self.create_publisher(PointCloud2, '/debug_w_avail_pc', 10)
        self.pub_z_targets = self.create_publisher(PointCloud2, '/debug_z_targets_pc', 10)
        self.pub_safe_mask = self.create_publisher(PointCloud2, '/debug_safe_spaces_pc', 10)

        # 3. Quadruped Kinematic Limits (in meters)
        # These define the physical constraints of your specific robot.
        self.z_min = 0.15  # Absolute lowest the robot's center of mass can crouch
        self.z_max = 0.40  # Absolute highest the robot can stand
        
        # Because it's a quadruped, height and width are inversely coupled:
        # Standing tall pulls the legs inward (narrow span).
        # Crouching low splays the legs outward (wide span).
        self.s_min = 0.40  # Narrowest width (occurs at z_max)
        self.s_max = 0.80  # Widest width (occurs at z_min)
        
        self.l_rob = 0.50  # Length of the robot (front to back)

    def get_layer(self, msg: GridMap, name: str) -> np.ndarray:
        """
        Extracts a specific data layer from the ROS GridMap message and reshapes 
        it from a flat 1D list back into a 2D numpy matrix.
        """
        idx = msg.layers.index(name)
        d   = msg.data[idx]
        n_cols = d.layout.dim[0].size
        n_rows = d.layout.dim[1].size
        return np.array(d.data, dtype=np.float32).reshape(n_cols, n_rows)

    def get_span_from_z(self, z_arr: np.ndarray) -> np.ndarray:
        """
        The Kinematic Mapping Function.
        Given an array of proposed Z heights, this calculates the required 
        horizontal leg span (S) for each height using a simple linear interpolation.
        
        Formula: S(Z) = S_max - [ (Z - Z_min) / (Z_max - Z_min) ] * (S_max - S_min)
        """
        ratio = (z_arr - self.z_min) / (self.z_max - self.z_min)
        return self.s_max - ratio * (self.s_max - self.s_min)

    def compute_clearances(self, floor: np.ndarray, ceil: np.ndarray, res: float):
        """
        Phase 1: Measure the Environment.
        Calculates the absolute physical space available at every grid cell.
        """
        # Create a boolean mask of "good" data. 
        # NaN means the camera couldn't see that spot (a shadow or hole).
        valid = ~np.isnan(floor) & ~np.isnan(ceil)

        # 1. Vertical Clearance (h_avail)
        # Simply: Ceiling Height - Floor Height.
        # If the cell is unobserved, we temporarily set the clearance to infinity so 
        # the math doesn't crash, but we will filter it out later using the 'valid' mask.
        clearance_raw = np.where(valid, ceil - floor, np.inf)
        
        # 2. Identify Walls
        # We strictly define a "wall" as any location where the vertical space 
        # is lower than the robot's absolute minimum crouch height. It is physically impassable.
        walls = valid & (clearance_raw < self.z_min)
        
        # 3. Horizontal Clearance (w_avail)
        # distance_transform_edt calculates the distance from every non-wall cell 
        # to the nearest wall cell. 
        # We multiply by 'res' (meters per cell) to convert grid units to real-world meters.
        # This tells us: "If I stand here, how far away is the nearest obstacle?"
        d_xy = distance_transform_edt(~walls) * res

        # Apply the valid mask. We don't want to guess clearances in camera blind spots.
        h_avail = np.where(valid, clearance_raw, np.nan)
        w_avail = np.where(valid, d_xy, np.nan) 
        
        return h_avail, w_avail, floor, valid

    def optimize_posture(self, h_avail: np.ndarray, w_avail: np.ndarray):
        """
        Phase 2: The Bottleneck Optimizer.
        For every grid cell, tests 15 postures and picks the safest one.
        Safest = the posture that maximizes the smallest gap between the robot and the terrain.
        """
        # Generate 15 candidate heights evenly spaced between min crouch and max stand
        z_candidates = np.linspace(self.z_min, self.z_max, 15)
        
        # Initialize tracking arrays. -inf means "no safe posture found yet"
        best_score = np.full_like(h_avail, -np.inf)
        best_z     = np.full_like(h_avail,  np.nan)

        for z in z_candidates:
            # How wide will the legs be at this height?
            s = self.get_span_from_z(z)
            
            # Margin = Buffer space remaining. 
            # How much air is above the head, and how much air is beside the legs?
            margin_z = h_avail - z
            margin_w = w_avail - s

            # Score this posture based purely on its tightest bottleneck.
            # If headroom is 0.05m but legroom is 1.0m, the score is 0.05.
            score = np.minimum(margin_z, margin_w)
            
            # HARD CONSTRAINT: If the robot's head clips the ceiling (Z > h_avail) 
            # or its legs clip the walls (S > w_avail), this posture is lethal.
            # Override the score to -infinity.
            score = np.where((z > h_avail) | (s > w_avail), -np.inf, score)

            # Did this posture score better than previous candidates?
            improved = score > best_score
            
            # Update the winning score and winning Z-height where it improved
            best_score = np.where(improved, score, best_score)
            best_z     = np.where(improved, z,     best_z)

        # If the best score is still -inf, it means ALL 15 postures resulted in a crash.
        # This grid cell is marked as unsafe.
        is_safe = best_score > -np.inf
        return best_z, is_safe

    def publish_pointcloud(self, publisher, header, info, values, base_z, mask, is_flat=True):
        """
        Utility to convert our 2D numpy math matrices into colored 3D PointClouds for RViz.
        """
        if not np.any(mask):
            return # Nothing to publish

        res = info.resolution
        # Map centers in real-world coordinates
        cx = info.pose.position.x
        cy = info.pose.position.y
        half_x = info.length_x / 2.0
        half_y = info.length_y / 2.0

        # Extract only the indices of cells that are valid/safe to process
        cols, rows = np.where(mask)
        
        # CONVERT GRID INDICES TO WORLD COORDINATES
        # Grid arrays are indexed top-to-bottom, left-to-right. 
        # We map these to standard ROS (X, Y) world coordinates relative to the map center.
        world_y = cy + half_y - (cols + 0.5) * res
        world_x = cx + half_x - (rows + 0.5) * res
        
        # The 'value' (clearance amount, score, boolean) will be passed to the 
        # 'intensity' channel of the point cloud so RViz can colorize it.
        intensities = values[cols, rows]
        
        # Determine the Z-height of the points
        if is_flat:
            # Force all points to Z=0. Creates a flat heatmap like a piece of paper.
            world_z = np.zeros_like(world_x)
        else:
            # Place the point in true 3D space: Floor height + half the robot's height.
            # This represents the actual center of mass of the robot.
            world_z = base_z[cols, rows] + (intensities / 2.0)

        # Construct the PointCloud2 byte payload
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        points = np.column_stack((world_x, world_y, world_z, intensities))
        cloud_msg = pc2.create_cloud(header, fields, points)
        publisher.publish(cloud_msg)

    def callback(self, floor_msg: GridMap, ceil_msg: GridMap):
        """
        The main pipeline loop. Triggered every time a matched pair of floor/ceil maps arrives.
        """
        if 'elevation' not in floor_msg.layers or 'elevation' not in ceil_msg.layers:
            return

        res   = floor_msg.info.resolution
        floor = self.get_layer(floor_msg, 'elevation')
        ceil  = self.get_layer(ceil_msg,  'elevation')

        # 1. Run the math
        h_avail, w_avail, f_base, valid = self.compute_clearances(floor, ceil, res)
        opt_z, safe_mask = self.optimize_posture(h_avail, w_avail)

        # 2. Log a quick health check to the terminal
        self.get_logger().info(f'Safe Cells: {np.sum(safe_mask)} / {np.sum(valid)}')

        # 3. Publish Debug Heatmaps
        # is_flat=True forces these to render as 2D color maps on the floor
        self.publish_pointcloud(self.pub_h_avail, floor_msg.header, floor_msg.info, h_avail, f_base, valid, is_flat=True)
        self.publish_pointcloud(self.pub_w_avail, floor_msg.header, floor_msg.info, w_avail, f_base, valid, is_flat=True)

        # 4. Publish 3D Z-Targets
        # is_flat=False means these dots will float in the air at the exact calculated height
        self.publish_pointcloud(self.pub_z_targets, floor_msg.header, floor_msg.info, opt_z, f_base, safe_mask, is_flat=False)
        
        # 5. Publish the Binary Safety Mask (1.0 = Safe, 0.0 = Unsafe)
        safe_float = safe_mask.astype(np.float32)
        self.publish_pointcloud(self.pub_safe_mask, floor_msg.header, floor_msg.info, safe_float, f_base, valid, is_flat=True)

def main(args=None):
    rclpy.init(args=args)
    node = PostureOptimizerDebug()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()