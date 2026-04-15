import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
import open3d as o3d
import time
from sensor_msgs_py import point_cloud2
import tf2_geometry_msgs
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from scipy.spatial.transform import Rotation


class CubePoseEstimator(Node):

    def __init__(self):
        super().__init__('cube_pose_estimator')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/fr3/depth_camera/points',
            self.callback,
            10
        )

        self.pose_pub = self.create_publisher(PoseStamped, '/cube_pose', 10)

        # TF
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.world_frame = 'world'

        # # Debug visualizer
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="Debug View")
        self.first_frame = True

    def callback(self, msg):
        callback_start = time.perf_counter()
        stage_start = callback_start

        # self.get_logger().info(
        #     f'[callback] start frame_stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
        # )

        # Convert to numpy
        points = np.array([
            [p[0], p[1], p[2]]
            for p in point_cloud2.read_points(msg, skip_nans=True)
        ])


        now = time.perf_counter()
        # self.get_logger().info(
        #     f'[callback] read_points={(now - stage_start) * 1000.0:.2f} ms '
        #     f'total={(now - callback_start) * 1000.0:.2f} ms n_points={len(points)}'
        # )
        stage_start = now

        if len(points) == 0:
            # self.get_logger().warn(
            #     f'[callback] no points in cloud; return total={(time.perf_counter() - callback_start) * 1000.0:.2f} ms'
            # )
            return

        # Open3D cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd = pcd.voxel_down_sample(voxel_size=0.005)

        now = time.perf_counter()
        # self.get_logger().info(
        #     f'[callback] voxel_down_sample={(now - stage_start) * 1000.0:.2f} ms '
        #     f'total={(now - callback_start) * 1000.0:.2f} ms n_downsampled={len(pcd.points)}'
        # )
        stage_start = now

        # Remove plane
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=0.02,
            ransac_n=3,
            num_iterations=500
        )
        pcd = pcd.select_by_index(inliers, invert=True)

        now = time.perf_counter()
        # self.get_logger().info(
        #     f'[callback] segment_plane={(now - stage_start) * 1000.0:.2f} ms '
        #     f'total={(now - callback_start) * 1000.0:.2f} ms removed={len(inliers)} remaining={len(pcd.points)}'
        # )
        stage_start = now

        # Clustering
        labels = np.array(pcd.cluster_dbscan(eps=0.01, min_points=30))
        now = time.perf_counter()
        # self.get_logger().info(
        #     f'[callback] cluster_dbscan={(now - stage_start) * 1000.0:.2f} ms '
        #     f'total={(now - callback_start) * 1000.0:.2f} ms n_labels={len(labels)}'
        # )
        stage_start = now

        if labels.max() < 0:
            # self.get_logger().debug(
            #     f'[callback] no clusters detected after plane removal; '
            #     f'return total={(time.perf_counter() - callback_start) * 1000.0:.2f} ms'
            # )
            return

        # Color clusters
        colors = np.zeros((len(labels), 3))
        for i in range(labels.max() + 1):
            colors[labels == i] = np.random.rand(3)

        pcd.colors = o3d.utility.Vector3dVector(colors)

        # Select largest cluster
        largest_cluster = max(set(labels), key=list(labels).count)
        indices = np.where(labels == largest_cluster)[0]
        cluster_points = pcd.select_by_index(indices)

        # Paint selected cluster RED
        cluster_points.paint_uniform_color([1, 0, 0])

        # Compute bounding box
        obb = cluster_points.get_oriented_bounding_box()
        obb.color = (0, 1, 0)

        center = obb.center
        R = obb.R
        quat = Rotation.from_matrix(R).as_quat()

        now = time.perf_counter()
        # self.get_logger().info(
        #     f'[callback] bbox_and_pose={(now - stage_start) * 1000.0:.2f} ms '
        #     f'total={(now - callback_start) * 1000.0:.2f} ms cluster_points={len(indices)}'
        # )
        stage_start = now

        # Pose in camera frame
        pose_camera = PoseStamped()
        pose_camera.header = msg.header
        pose_camera.pose.position.x = center[0]
        pose_camera.pose.position.y = center[1]
        pose_camera.pose.position.z = center[2]
        pose_camera.pose.orientation.x = quat[0]
        pose_camera.pose.orientation.y = quat[1]
        pose_camera.pose.orientation.z = quat[2]
        pose_camera.pose.orientation.w = quat[3]

        # Transform to world
        try:
            pose_world = self.tf_buffer.transform(
                pose_camera,
                self.world_frame,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # self.get_logger().warn(
            #     f'[callback] transform failed to {self.world_frame}: {e}; '
            #     f'return total={(time.perf_counter() - callback_start) * 1000.0:.2f} ms'
            # )
            return

        now = time.perf_counter()
        # self.get_logger().info(
        #     f'[callback] tf_transform={(now - stage_start) * 1000.0:.2f} ms '
        #     f'total={(now - callback_start) * 1000.0:.2f} ms'
        # )
        stage_start = now

        self.pose_pub.publish(pose_world)

        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id = 'cube'
        t.transform.translation.x = pose_world.pose.position.x
        t.transform.translation.y = pose_world.pose.position.y
        t.transform.translation.z = pose_world.pose.position.z
        t.transform.rotation = pose_world.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        now = time.perf_counter()
        # self.get_logger().info(
        #     f'[callback] publish_and_broadcast={(now - stage_start) * 1000.0:.2f} ms '
        #     f'total={(now - callback_start) * 1000.0:.2f} ms'
        # )
        stage_start = now

        # -----------------------
        # DEBUG VISUALIZATION
        # -----------------------
        self.vis.clear_geometries()
        self.vis.add_geometry(pcd)              # all clusters
        self.vis.add_geometry(cluster_points)   # selected cluster
        self.vis.add_geometry(obb)              # bounding box

        self.vis.poll_events()
        self.vis.update_renderer()

        now = time.perf_counter()
        # self.get_logger().info(
        #     f'[callback] visualization={(now - stage_start) * 1000.0:.2f} ms '
        #     f'total={(now - callback_start) * 1000.0:.2f} ms'
        # )


def main(args=None):
    rclpy.init(args=args)
    node = CubePoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


###################################################################################################################



# import rclpy
# from rclpy.duration import Duration
# from rclpy.node import Node

# import cv2
# import numpy as np
# from cv_bridge import CvBridge
# from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
# from sensor_msgs.msg import CameraInfo, Image, PointCloud2
# from sensor_msgs_py import point_cloud2
# import message_filters
# import tf2_geometry_msgs  # noqa: F401
# from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


# class CubePoseEstimator(Node):

#     def __init__(self):
#         super().__init__('cube_pose_estimator')

#         self.pose_pub = self.create_publisher(PoseStamped, '/cube_pose', 10)
#         self.debug_image_pub = self.create_publisher(Image, '/cube_pose/debug_image', 10)

#         self.bridge = CvBridge()
#         self.tf_broadcaster = TransformBroadcaster(self)
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         self.target_frame = self.declare_parameter('target_frame', 'world').value
#         self.camera_frame = self.declare_parameter('camera_frame', 'fr3/rgbd_camera_frame').value
#         self.image_topic = self.declare_parameter('image_topic', '/fr3/depth_camera/image').value
#         self.camera_info_topic = self.declare_parameter('camera_info_topic', '/fr3/depth_camera/camera_info').value
#         self.object_plane_z = self.declare_parameter('object_plane_z', 0.1).value
#         self.ray_frame_convention = self.declare_parameter('ray_frame_convention', 'optical').value

#         self.fx = self.declare_parameter('fx', 277.1).value
#         self.fy = self.declare_parameter('fy', 277.1).value
#         self.cx = self.declare_parameter('cx', 160.5).value
#         self.cy = self.declare_parameter('cy', 120.5).value

#         self.dist_coeffs = self.declare_parameter('distortion_coefficients', [0.104207, -0.255558, 0.000512, 0.000319, 0.128694]).value
#         self.camera_info_received = False

#         self.image_subscription = message_filters.Subscriber(self, Image, self.image_topic)
#         self.pointcloud_subscription = message_filters.Subscriber(self, PointCloud2, '/fr3/depth_camera/points')
#         self.sync = message_filters.ApproximateTimeSynchronizer(
#             [self.image_subscription, self.pointcloud_subscription],
#             queue_size=10,
#             slop=0.1,
#         )
#         self.sync.registerCallback(self.synced_callback)

#         self.camera_info_subscription = self.create_subscription(
#             CameraInfo,
#             self.camera_info_topic,
#             self.camera_info_callback,
#             10,
#         )
        
#         self.get_logger().info('2D cube pose estimator initialized')
#         self.get_logger().info(f'Target frame for transformation: {self.target_frame}')
#         self.get_logger().info(f'Camera frame override: {self.camera_frame or "from image header"}')
#         self.get_logger().info(f'Using plane z={self.object_plane_z} in {self.target_frame}')
#         self.get_logger().info(f'Ray frame convention: {self.ray_frame_convention}')
#         self.get_logger().info(f'Camera info topic: {self.camera_info_topic}')
#         self.get_logger().info(f'Camera intrinsics: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}')

#     def synced_callback(self, image_msg, pointcloud_msg):
#         self.image_callback(image_msg, pointcloud_msg)

#     def camera_info_callback(self, msg):
#         fx = float(msg.k[0])
#         fy = float(msg.k[4])
#         cx = float(msg.k[2])
#         cy = float(msg.k[5])

#         if fx <= 0.0 or fy <= 0.0:
#             return

#         self.fx = fx
#         self.fy = fy
#         self.cx = cx
#         self.cy = cy

#         if not self.camera_info_received:
#             self.camera_info_received = True
#             self.get_logger().info(
#                 f'CameraInfo intrinsics loaded: fx={self.fx:.3f}, fy={self.fy:.3f}, '
#                 f'cx={self.cx:.3f}, cy={self.cy:.3f} ({msg.width}x{msg.height})'
#             )

#     def image_callback(self, msg, pointcloud_msg=None):
#         try:
#             self.get_logger().debug(f'Image received: encoding={msg.encoding}, frame_id={msg.header.frame_id}')
#             if msg.encoding == 'mono8' or msg.encoding == 'L8':
#                 mono = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
#                 image = cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
#             elif msg.encoding == 'rgb8':
#                 rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
#                 image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
#             elif msg.encoding == 'bgr8':
#                 image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#             else:
#                 image = self.bridge.imgmsg_to_cv2(msg)
#                 if len(image.shape) == 2:
#                     image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
#                 else:
#                     image = image.copy()
#         except Exception as e:
#             self.get_logger().error(f'Image conversion error: {e}')
#             return

#         detection = self.segment_cube_from_image(image)
#         if detection is None:
#             self.get_logger().debug('No cube pixels detected in the current frame')
#             return

#         mask, bbox, centroid = detection
#         pixel_u, pixel_v = centroid

#         source_frame = self.camera_frame if self.camera_frame else msg.header.frame_id
#         if not source_frame:
#             self.get_logger().warn('No camera frame available for TF lookup')
#             return

#         pose_world = self.pose_from_pointcloud(pixel_u, pixel_v, pointcloud_msg)
#         if pose_world is None:
#             try:
#                 pose_world = self.project_pixel_to_world(pixel_u, pixel_v, source_frame, msg.header.stamp)
#             except TransformException as e:
#                 self.get_logger().warn(f'TF lookup failed: {e}')
#                 return

#         self.pose_pub.publish(pose_world)
#         self.publish_debug_image(image, mask, bbox, centroid)

#         transform = TransformStamped()
#         transform.header.stamp = pose_world.header.stamp
#         transform.header.frame_id = pose_world.header.frame_id
#         transform.child_frame_id = 'cube'
#         transform.transform.translation.x = pose_world.pose.position.x
#         transform.transform.translation.y = pose_world.pose.position.y
#         transform.transform.translation.z = pose_world.pose.position.z
#         transform.transform.rotation.x = pose_world.pose.orientation.x
#         transform.transform.rotation.y = pose_world.pose.orientation.y
#         transform.transform.rotation.z = pose_world.pose.orientation.z
#         transform.transform.rotation.w = pose_world.pose.orientation.w
#         self.tf_broadcaster.sendTransform(transform)

#     def pose_from_pointcloud(self, pixel_u, pixel_v, cloud_msg):
#         """Estimate cube pose from the point cloud around the detected 2D centroid."""
#         if cloud_msg is None:
#             return None

#         center_u = int(round(pixel_u))
#         center_v = int(round(pixel_v))
#         search_radius = 2

#         uvs = []
#         for du in range(-search_radius, search_radius + 1):
#             for dv in range(-search_radius, search_radius + 1):
#                 uvs.append((center_u + du, center_v + dv))

#         try:
#             sampled_points = list(point_cloud2.read_points(
#                 cloud_msg,
#                 field_names=('x', 'y', 'z'),
#                 skip_nans=True,
#                 uvs=uvs,
#             ))
#         except Exception as e:
#             self.get_logger().debug(f'Pointcloud sampling failed: {e}')
#             return None

#         if not sampled_points:
#             return None

#         sampled_points = np.array(sampled_points, dtype=np.float64)
#         median_point = np.median(sampled_points, axis=0)

#         self.get_logger().info(
#             f'Cube camera frame=({median_point[0]:.3f}, {median_point[1]:.3f}, {median_point[2]:.3f}) '
#             f'for centroid=({pixel_u:.1f}, {pixel_v:.1f})'
#         )

#         point_cam = PointStamped()
#         point_cam.header.frame_id = cloud_msg.header.frame_id or self.camera_frame
#         point_cam.header.stamp = cloud_msg.header.stamp
#         point_cam.point.x = float(median_point[0])
#         point_cam.point.y = float(median_point[1])
#         point_cam.point.z = float(median_point[2])

#         try:
#             point_world = self.tf_buffer.transform(
#                 point_cam,
#                 self.target_frame,
#                 timeout=Duration(seconds=0.2),
#             )
#         except TransformException as e:
#             self.get_logger().debug(f'Pointcloud transform failed: {e}')
#             return None

#         pose_world = PoseStamped()
#         pose_world.header.frame_id = self.target_frame
#         pose_world.header.stamp = point_cam.header.stamp
#         pose_world.pose.position.x = float(point_world.point.x)
#         pose_world.pose.position.y = float(point_world.point.y)
#         pose_world.pose.position.z = float(point_world.point.z)
#         pose_world.pose.orientation.x = 0.0
#         pose_world.pose.orientation.y = 0.0
#         pose_world.pose.orientation.z = 0.0
#         pose_world.pose.orientation.w = 1.0

#         self.get_logger().info(
#             f'Cube centroid=({pixel_u:.1f}, {pixel_v:.1f}) depth-median=('
#             f'{median_point[0]:.3f}, {median_point[1]:.3f}, {median_point[2]:.3f}) -> '
#             f'world=({pose_world.pose.position.x:.3f}, {pose_world.pose.position.y:.3f}, {pose_world.pose.position.z:.3f})'
#         )

#         return pose_world

#     def project_pixel_to_world(self, pixel_u, pixel_v, source_frame, stamp):
        
#         K = np.array([
#         [self.fx, 0.0, self.cx],
#         [0.0, self.fy, self.cy],
#         [0.0, 0.0, 1.0]
#         ])

#         D = np.array(self.dist_coeffs)  # [k1, k2, p1, p2, k3]

#         pts = np.array([[[pixel_u, pixel_v]]], dtype=np.float32)

#         undistorted = cv2.undistortPoints(pts, K, D, P=K)

#         u = undistorted[0, 0, 0]
#         v = undistorted[0, 0, 1]

#         """Project a 2D image point onto the fixed world plane."""
#         ray_x = (u - self.cx) / self.fx * 0.5
#         ray_y = (v - self.cy) / self.fy * 0.5

#         header_stamp = stamp if stamp.sec != 0 or stamp.nanosec != 0 else self.get_clock().now().to_msg()

#         origin_cam = PointStamped()
#         origin_cam.header.frame_id = source_frame
#         origin_cam.header.stamp = header_stamp
#         origin_cam.point.x = 0.0
#         origin_cam.point.y = 0.0
#         origin_cam.point.z = 0.0

#         ray_cam = PointStamped()
#         ray_cam.header.frame_id = source_frame
#         ray_cam.header.stamp = header_stamp

#         # Intrinsics produce an optical-frame ray (x right, y down, z forward).
#         # Gazebo camera links are commonly x forward, y left, z up.

#         # if self.ray_frame_convention == 'optical':
#         ray_cam.point.x = ray_x
#         ray_cam.point.y = ray_y
#         ray_cam.point.z = 0.5




#         print(f'Computed ray in camera frame: ({ray_cam.point.x:.3f}, {ray_cam.point.y:.3f}, {ray_cam.point.z:.3f})')
#         # elif self.ray_frame_convention == 'gazebo':
#         # ray_cam.point.x = 1.0
#         # ray_cam.point.y = -ray_x
#         # ray_cam.point.z = -ray_y
#         # elif self.ray_frame_convention == 'x_down_y_left_z_forward':
#             # Camera frame: X down, Y left, Z forward
#         # ray_cam.point.x = ray_y      # optical Y (down) → camera X (down)
#         # ray_cam.point.y = -ray_x     # optical X (right) → camera Y (left)
#         # ray_cam.point.z = 1.0        # optical Z (forward) → camera Z (forward)
#         # else:
#             # raise ValueError(f"Unknown ray frame convention: {self.ray_frame_convention}")

#         # if self.ray_frame_convention == 'gazebo':
#         #     ray_cam.point.x = 1.0
#         #     ray_cam.point.y = -ray_x
#         #     ray_cam.point.z = -ray_y
#         # else:
#         #     ray_cam.point.x = ray_x
#         #     ray_cam.point.y = ray_y
#         #     ray_cam.point.z = 1.0

#         origin_world = self.tf_buffer.transform(
#             origin_cam,
#             self.target_frame,
#             timeout=Duration(seconds=0.2),
#         )
#         ray_world = self.tf_buffer.transform(
#             ray_cam,
#             self.target_frame,
#             timeout=Duration(seconds=0.2),
#         )

#         direction_world = np.array([
#             ray_world.point.x - origin_world.point.x,
#             ray_world.point.y - origin_world.point.y,
#             ray_world.point.z - origin_world.point.z,
#         ])

#         if abs(direction_world[2]) < 1e-9:
#             raise TransformException('Ray is parallel to the target plane')

#         scale = (self.object_plane_z - origin_world.point.z) / direction_world[2]
#         world_x = origin_world.point.x + scale * direction_world[0]
#         world_y = origin_world.point.y + scale * direction_world[1]

#         pose_world = PoseStamped()
#         pose_world.header.frame_id = self.target_frame
#         pose_world.header.stamp = header_stamp
#         pose_world.pose.position.x = float(world_x)
#         pose_world.pose.position.y = float(world_y)
#         pose_world.pose.position.z = float(self.object_plane_z)
#         pose_world.pose.orientation.x = 0.0
#         pose_world.pose.orientation.y = 0.0
#         pose_world.pose.orientation.z = 0.0
#         pose_world.pose.orientation.w = 1.0

#         self.get_logger().info(
#             f'Cube centroid=({pixel_u:.1f}, {pixel_v:.1f}) -> '
#             f'world=({world_x:.3f}, {world_y:.3f}, {self.object_plane_z:.3f})'
#         )

#         return pose_world

#     def segment_cube_from_image(self, image):
#         """Segment the cube and return a binary mask, bounding box, and centroid."""
#         if len(image.shape) == 3 and image.shape[2] == 3:
#             hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#             lower_red_1 = np.array([0, 70, 50])
#             upper_red_1 = np.array([10, 255, 255])
#             lower_red_2 = np.array([170, 70, 50])
#             upper_red_2 = np.array([180, 255, 255])

#             mask = cv2.inRange(hsv, lower_red_1, upper_red_1)
#             mask |= cv2.inRange(hsv, lower_red_2, upper_red_2)
#         else:
#             gray = image if len(image.shape) == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#             blurred = cv2.GaussianBlur(gray, (5, 5), 0)
#             edges = cv2.Canny(blurred, 20, 80)
#             mask = edges

#         kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
#         mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
#         mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
#         mask = cv2.dilate(mask, kernel, iterations=2)

#         pixel_rows, pixel_cols = np.where(mask > 0)
#         if pixel_rows.size == 0:
#             self.get_logger().debug('Segmentation mask is empty after morphology')
#             return None

#         x_min = int(np.min(pixel_cols))
#         x_max = int(np.max(pixel_cols))
#         y_min = int(np.min(pixel_rows))
#         y_max = int(np.max(pixel_rows))
#         centroid_u = float(np.mean(pixel_cols))
#         centroid_v = float(np.mean(pixel_rows))

#         bbox = (x_min, y_min, x_max - x_min + 1, y_max - y_min + 1)
#         return mask, bbox, (centroid_u, centroid_v)

#     def publish_debug_image(self, image, mask, bbox, centroid):
#         debug_img = image.copy()
#         if len(debug_img.shape) == 2:
#             debug_img = cv2.cvtColor(debug_img, cv2.COLOR_GRAY2BGR)

#         x, y, w, h = bbox
#         pixel_u, pixel_v = centroid
#         cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
#         cv2.circle(debug_img, (int(pixel_u), int(pixel_v)), 4, (255, 0, 0), -1)
#         cv2.putText(debug_img, 'Cube', (x, max(0, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

#         mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
#         overlay = np.where(mask_bgr > 0, [0, 0, 255], [0, 0, 0]).astype(np.uint8)
#         debug_img = cv2.addWeighted(debug_img, 0.8, overlay, 0.2, 0)

#         try:
#             debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
#             debug_msg.header.stamp = self.get_clock().now().to_msg()
#             debug_msg.header.frame_id = 'world'
#             self.debug_image_pub.publish(debug_msg)
#         except Exception as e:
#             self.get_logger().error(f'Error publishing debug image: {e}')



# def main(args=None):
#     rclpy.init(args=args)
#     node = CubePoseEstimator()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


if __name__ == '__main__':
    main()