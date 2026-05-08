"""
Medicine detector with fixed camera behind shelf.

Detects ALL medicines from stable point-cloud view, picks the one closest
to the gripper EEF, and overlays green contours on the live colour image.
"""

import math
import threading
import numpy as np
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from builtin_interfaces.msg import Duration as DurationMsg
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Vector3
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from sklearn.cluster import DBSCAN


class MedicineDetector(Node):

    def __init__(self):
        super().__init__('medicine_detector')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._bridge = CvBridge()

        # latest colour image and camera intrinsics (updated from callbacks)
        self._latest_image: Image = None
        self._camera_info: CameraInfo = None
        self._img_lock = threading.Lock()

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(
            PointCloud2,
            '/camera_head/depth/color/points',
            self._cloud_cb,
            qos_profile_sensor_data)

        self.create_subscription(
            Image,
            '/camera_head/color/image_raw',
            self._image_cb,
            qos_profile_sensor_data)

        self.create_subscription(
            CameraInfo,
            '/camera_head/depth/camera_info',
            self._camera_info_cb,
            qos_profile_sensor_data)

        # ── Publishers ────────────────────────────────────────────────────
        self.poses_pub   = self.create_publisher(PoseArray,    '/detected_medicines',          10)
        self.target_pub  = self.create_publisher(PoseStamped,  '/target_medicine_pose',         10)
        self.markers_pub = self.create_publisher(MarkerArray,  '/detected_medicines_markers',   10)
        self.image_pub   = self.create_publisher(Image,        '/medicine_detection_image',     10)

        # ── DBSCAN parameters ─────────────────────────────────────────────
        self._eps         = 0.030   # 3 cm cluster radius
        self._min_samples = 8

        # ── Approach orientation: gripper faces +Y (toward shelf) ─────────
        self._ori_x = -math.sin(math.pi / 4.0)
        self._ori_y = 0.0
        self._ori_z = 0.0
        self._ori_w =  math.cos(math.pi / 4.0)

        self._gripper_frame  = 'link6_flange'
        self._debug_logged   = False

        self.get_logger().info(
            'Medicine detector started.\n'
            f'  DBSCAN eps={self._eps:.3f} m  min_samples={self._min_samples}\n'
            f'  Target = closest medicine to {self._gripper_frame}')

    # ── Simple sensor callbacks ───────────────────────────────────────────

    def _image_cb(self, msg: Image):
        with self._img_lock:
            self._latest_image = msg

    def _camera_info_cb(self, msg: CameraInfo):
        with self._img_lock:
            self._camera_info = msg

    # ── Main point-cloud callback ─────────────────────────────────────────

    def _cloud_cb(self, msg: PointCloud2):
        source_frame = msg.header.frame_id or 'camera_head_link'

        # 1. TF: camera → world
        try:
            tf = self.tf_buffer.lookup_transform(
                'world', source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except Exception as exc:
            self.get_logger().warn(
                f'TF world←{source_frame}: {exc}', throttle_duration_sec=5.0)
            return

        # 2. Read point cloud
        try:
            raw = list(pc2.read_points(msg, field_names=('x', 'y', 'z'),
                                       skip_nans=True))
            if len(raw) < 50:
                return
            pts = np.array([[p[0], p[1], p[2]] for p in raw], dtype=np.float64)
        except Exception as exc:
            self.get_logger().warn(f'Cloud read: {exc}', throttle_duration_sec=5.0)
            return

        # 3. Transform to world frame
        t  = tf.transform.translation
        r  = tf.transform.rotation
        qx, qy, qz, qw = r.x, r.y, r.z, r.w
        rot = np.array([
            [1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
            [  2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz),   2*(qy*qz-qx*qw)],
            [  2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
        ])
        pts_w = (rot @ pts.T).T + np.array([t.x, t.y, t.z])

        if not self._debug_logged:
            self.get_logger().info(
                f'World cloud ({len(pts_w)} pts): '
                f'x∈[{pts_w[:,0].min():.3f},{pts_w[:,0].max():.3f}] '
                f'y∈[{pts_w[:,1].min():.3f},{pts_w[:,1].max():.3f}] '
                f'z∈[{pts_w[:,2].min():.3f},{pts_w[:,2].max():.3f}]')
            self._debug_logged = True

        # 4. ROI filter — shelf region in TF world (= base_link) frame
        x_min, x_max = -0.13,  0.13   # medicines span ±0.09 m laterally
        y_min, y_max =  0.18,  0.35   # shelf depth ≈ 0.265 m from robot base
        z_min, z_max =  0.17,  0.39   # z_max covers tallest medicine top
        mask = ((pts_w[:, 0] >= x_min) & (pts_w[:, 0] <= x_max) &
                (pts_w[:, 1] >= y_min) & (pts_w[:, 1] <= y_max) &
                (pts_w[:, 2] >= z_min) & (pts_w[:, 2] <= z_max))
        shelf_pts = pts_w[mask]

        # Exclude the upper shelf board surface (TF z ≈ 0.265–0.283 m).
        # The board top is at z=0.280m; medicines sit above it from z=0.280m.
        # Without this, DBSCAN merges board points with medicine bottoms,
        # creating a cluster too large to pass the bounding-box filter.
        upper_board = ((shelf_pts[:, 2] >= 0.265) & (shelf_pts[:, 2] <= 0.283))
        shelf_pts = shelf_pts[~upper_board]

        if len(shelf_pts) < 30:
            self.get_logger().info(
                f'No points in shelf ROI ({len(shelf_pts)} pts)',
                throttle_duration_sec=3.0)
            return

        self.get_logger().info(
            f'Shelf ROI: {len(shelf_pts)} pts', throttle_duration_sec=2.0)

        # 5. DBSCAN on shelf points only
        clustering = DBSCAN(eps=self._eps, min_samples=self._min_samples).fit(shelf_pts)
        labels     = clustering.labels_

        cluster_ids = set(labels) - {-1}
        if not cluster_ids:
            self.get_logger().info('No medicine clusters', throttle_duration_sec=3.0)
            return

        raw_clusters = [shelf_pts[labels == cid] for cid in cluster_ids]

        # Reject shelf structures and flat surfaces:
        #   1. Bounding box max ≤ 15 cm (medicine packets)
        #   2. Minimum z-span ≥ 3 cm (shelf boards are only 12 mm thick)
        #   3. Centroid y ≤ 0.30 m (medicine fronts; board tops are deeper)
        clusters = []
        for c in raw_clusters:
            bb = c.max(axis=0) - c.min(axis=0)
            centroid_y = float(np.mean(c[:, 1]))
            if (np.all(bb <= 0.15) and
                    bb[2] >= 0.030 and
                    centroid_y <= 0.30):
                clusters.append(c)
            else:
                self.get_logger().debug(
                    f'Rejected cluster bbox={bb} cy={centroid_y:.3f}',
                    throttle_duration_sec=2.0)

        if not clusters:
            self.get_logger().info(
                'No medicine clusters after size filter', throttle_duration_sec=3.0)
            return

        self.get_logger().info(
            f'Detected {len(clusters)} medicine cluster(s)', throttle_duration_sec=2.0)

        # 6. Get gripper position
        try:
            g_tf = self.tf_buffer.lookup_transform(
                'world', self._gripper_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.05))
            gripper_pos = np.array([
                g_tf.transform.translation.x,
                g_tf.transform.translation.y,
                g_tf.transform.translation.z])
        except Exception as exc:
            self.get_logger().warn(
                f'TF {self._gripper_frame}: {exc}', throttle_duration_sec=5.0)
            return

        # 7. Centroids + gripper-relative distances → pick closest
        centroids = [np.mean(c, axis=0) for c in clusters]
        distances = [np.linalg.norm(ctr - gripper_pos) for ctr in centroids]
        best_idx  = int(np.argmin(distances))
        best      = centroids[best_idx]
        best_dist = distances[best_idx]

        # 8. Publish poses + target
        now = self.get_clock().now().to_msg()

        arr = PoseArray()
        arr.header.frame_id = 'world'
        arr.header.stamp = now
        for c in centroids:
            arr.poses.append(self._centroid_pose(c))
        self.poses_pub.publish(arr)

        ps = PoseStamped()
        ps.header.frame_id = 'world'
        ps.header.stamp = now
        ps.pose = self._centroid_pose(best)
        self.target_pub.publish(ps)

        self.get_logger().info(
            f'TARGET: ({best[0]:.3f}, {best[1]:.3f}, {best[2]:.3f}) '
            f'dist={best_dist:.3f} m',
            throttle_duration_sec=2.0)

        # 9. RViz markers
        self._publish_markers(now, clusters, centroids, distances, best_idx, gripper_pos)

        # 10. Colour image overlay with contours
        self._publish_image_overlay(clusters, centroids, best_idx)

    # ── Helpers ───────────────────────────────────────────────────────────

    def _centroid_pose(self, c: np.ndarray) -> Pose:
        p = Pose()
        p.position.x, p.position.y, p.position.z = float(c[0]), float(c[1]), float(c[2])
        p.orientation.x = self._ori_x
        p.orientation.y = self._ori_y
        p.orientation.z = self._ori_z
        p.orientation.w = self._ori_w
        return p

    # ── Image overlay ─────────────────────────────────────────────────────

    def _publish_image_overlay(self, clusters, centroids, best_idx):
        with self._img_lock:
            img_msg  = self._latest_image
            cam_info = self._camera_info
        if img_msg is None or cam_info is None:
            return

        # TF: world → camera optical frame
        try:
            tf_cam = self.tf_buffer.lookup_transform(
                'camera_head_depth_optical_frame', 'world',
                rclpy.time.Time(), timeout=Duration(seconds=0.05))
        except Exception as exc:
            self.get_logger().warn(
                f'TF cam←world: {exc}', throttle_duration_sec=5.0)
            return

        # Build rotation + translation from TF
        r  = tf_cam.transform.rotation
        t  = tf_cam.transform.translation
        qx, qy, qz, qw = r.x, r.y, r.z, r.w
        R_cw = np.array([
            [1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
            [  2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz),   2*(qy*qz-qx*qw)],
            [  2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
        ])
        t_cw = np.array([t.x, t.y, t.z])

        # Camera intrinsics
        K  = np.array(cam_info.k).reshape(3, 3)
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        # Convert image to OpenCV
        try:
            img = self._bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge: {exc}', throttle_duration_sec=5.0)
            return

        h, w = img.shape[:2]

        for i, (cluster, centroid) in enumerate(zip(clusters, centroids)):
            is_target = (i == best_idx)

            # Transform 3-D cluster points → camera frame
            pts_cam = (R_cw @ cluster.T).T + t_cw

            # Keep only points in front of camera
            front = pts_cam[:, 2] > 0.01
            if front.sum() < 3:
                continue
            pts_cam = pts_cam[front]

            # Project to pixel coordinates
            u = (fx * pts_cam[:, 0] / pts_cam[:, 2] + cx).astype(np.int32)
            v = (fy * pts_cam[:, 1] / pts_cam[:, 2] + cy).astype(np.int32)

            # Clip to image bounds
            valid = (u >= 0) & (u < w) & (v >= 0) & (v < h)
            if valid.sum() < 3:
                continue

            pts_2d = np.column_stack([u[valid], v[valid]])

            # Tight axis-aligned bounding rect around projected pixels
            u_min, u_max = int(pts_2d[:, 0].min()), int(pts_2d[:, 0].max())
            v_min, v_max = int(pts_2d[:, 1].min()), int(pts_2d[:, 1].max())

            if is_target:
                color     = (0, 255, 0)   # bright green for target
                thickness = 3
            else:
                color     = (0, 165, 255) # orange for others
                thickness = 2

            cv2.rectangle(img, (u_min, v_min), (u_max, v_max),
                          color, thickness, lineType=cv2.LINE_AA)

            # Project centroid for label
            ctr_cam = R_cw @ centroid + t_cw
            if ctr_cam[2] > 0.01:
                uc = int(fx * ctr_cam[0] / ctr_cam[2] + cx)
                vc = int(fy * ctr_cam[1] / ctr_cam[2] + cy)
                if 0 <= uc < w and 0 <= vc < h:
                    label = 'TARGET' if is_target else f'med{i}'
                    cv2.putText(img, label, (uc - 30, vc - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2,
                                cv2.LINE_AA)

        # Publish annotated image
        try:
            out = self._bridge.cv2_to_imgmsg(img, 'bgr8')
            out.header = img_msg.header
            self.image_pub.publish(out)
        except Exception as exc:
            self.get_logger().warn(f'publish image: {exc}', throttle_duration_sec=5.0)

    # ── RViz markers ──────────────────────────────────────────────────────

    def _publish_markers(self, stamp, clusters, centroids, distances,
                         target_idx, gripper_pos):
        arr = MarkerArray()

        for i, (cluster, centroid, dist) in enumerate(
                zip(clusters, centroids, distances)):
            is_target = (i == target_idx)
            min_pt    = cluster.min(axis=0)
            max_pt    = cluster.max(axis=0)
            size      = np.maximum(max_pt - min_pt, 0.02)

            # Bounding box cube
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp    = stamp
            m.id              = i
            m.type            = Marker.CUBE
            m.action          = Marker.ADD
            m.pose.position.x = float(centroid[0])
            m.pose.position.y = float(centroid[1])
            m.pose.position.z = float(centroid[2])
            m.pose.orientation.w = 1.0
            m.scale           = Vector3(x=float(size[0]), y=float(size[1]),
                                        z=float(size[2]))
            m.color           = (ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.7)
                                 if is_target else
                                 ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.4))
            m.lifetime        = DurationMsg(sec=2, nanosec=0)   # ← fixed
            arr.markers.append(m)

            # Distance text above target
            if is_target:
                tm = Marker()
                tm.header.frame_id = 'world'
                tm.header.stamp    = stamp
                tm.id              = 100 + i
                tm.type            = Marker.TEXT_VIEW_FACING
                tm.action          = Marker.ADD
                tm.pose.position.x = float(centroid[0])
                tm.pose.position.y = float(centroid[1])
                tm.pose.position.z = float(centroid[2] + 0.08)
                tm.pose.orientation.w = 1.0
                tm.scale.z         = 0.04
                tm.color           = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                tm.text            = f'CLOSEST\n{dist:.2f}m'
                tm.lifetime        = DurationMsg(sec=2, nanosec=0)   # ← fixed
                arr.markers.append(tm)

        # Gripper sphere
        gm = Marker()
        gm.header.frame_id = 'world'
        gm.header.stamp    = stamp
        gm.id              = 200
        gm.type            = Marker.SPHERE
        gm.action          = Marker.ADD
        gm.pose.position.x = float(gripper_pos[0])
        gm.pose.position.y = float(gripper_pos[1])
        gm.pose.position.z = float(gripper_pos[2])
        gm.pose.orientation.w = 1.0
        gm.scale           = Vector3(x=0.03, y=0.03, z=0.03)
        gm.color           = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        gm.lifetime        = DurationMsg(sec=2, nanosec=0)   # ← fixed
        arr.markers.append(gm)

        self.markers_pub.publish(arr)


def main():
    rclpy.init()
    node = MedicineDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
