"""Node target_pose_node: /tracked_target (pixel) + /camera/depth_image +
/camera/camera_info -> diem 3D trong khung "trunk" -> /target_pose.

Cong thuc pinhole (quadruped_ros_system.md Sec 5):
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    Z = depth
Ket qua nay theo quy uoc "optical frame" (z-forward, x-right, y-down, REP-103),
KHONG phai khung camera_link vat ly (x-forward giong than robot) - vi vay gan
cung frame_id="camera_link_optical" (link rieng trong robot.xacro, xoay san
tu camera_link) roi moi tf2-transform sang "trunk", thay vi tin frame_id ma
Gazebo tu dat cho topic depth/camera_info (co the khac ten link URDF).
"""
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener

from quadruped_interfaces.msg import TrackedTarget

SOURCE_FRAME = 'camera_link_optical'
TARGET_FRAME = 'trunk'


class TargetPoseNode(Node):
    def __init__(self):
        super().__init__('target_pose_node')

        self.bridge = CvBridge()
        self._depth = None       # cv2 array 32FC1, khung camera
        self._intrinsics = None  # (fx, fy, cx, cy)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PointStamped, '/target_pose', 10)

        self.create_subscription(Image, '/camera/depth_image', self._on_depth, 5)
        self.create_subscription(CameraInfo, '/camera/camera_info', self._on_camera_info, 5)
        self.create_subscription(TrackedTarget, '/tracked_target', self._on_target, 10)

        self.get_logger().info('target_pose_node san sang')

    def _on_depth(self, msg: Image):
        self._depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    def _on_camera_info(self, msg: CameraInfo):
        k = msg.k
        self._intrinsics = (k[0], k[4], k[2], k[5])

    def _on_target(self, msg: TrackedTarget):
        if self._depth is None or self._intrinsics is None:
            return

        u = int(round(msg.center_x))
        v = int(round(msg.center_y))
        h, w = self._depth.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            return

        depth = float(self._depth[v, u])
        if not (depth > 0.0) or depth != depth:  # <= 0 hoac NaN -> khong hop le
            return

        fx, fy, cx, cy = self._intrinsics
        point = PointStamped()
        point.header.frame_id = SOURCE_FRAME
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x = (u - cx) * depth / fx
        point.point.y = (v - cy) * depth / fy
        point.point.z = depth

        try:
            transform = self.tf_buffer.lookup_transform(
                TARGET_FRAME, SOURCE_FRAME, rclpy.time.Time()
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Khong lookup duoc tf {SOURCE_FRAME} -> {TARGET_FRAME}: {exc}',
                throttle_duration_sec=5.0,
            )
            return

        self.pub.publish(do_transform_point(point, transform))


def main(args=None):
    rclpy.init(args=args)
    node = TargetPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
