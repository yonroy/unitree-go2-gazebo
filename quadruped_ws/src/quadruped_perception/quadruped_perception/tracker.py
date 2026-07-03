"""Node tracker: /detections (vision_msgs/Detection2DArray, tu object_detector)
-> giu 1 target on dinh -> /tracked_target (quadruped_interfaces/TrackedTarget).

Logic don gian (khong dung ByteTrack, chi phu hop scene 1 target): moi frame
chon detection GAN TAM BBOX FRAME TRUOC nhat (neu da co target va nam trong
nguong max_jump_px), nguoc lai (chua co target, hoac khong detection nao du
gan) thi chon detection confidence cao nhat.
"""
import math

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

from quadruped_interfaces.msg import TrackedTarget


class Tracker(Node):
    def __init__(self):
        super().__init__('tracker')

        self.declare_parameter('max_jump_px', 150.0)
        self._max_jump_px = self.get_parameter('max_jump_px').value
        self._last_center = None  # (x, y) pixel, frame truoc

        self.pub = self.create_publisher(TrackedTarget, '/tracked_target', 10)
        self.sub = self.create_subscription(
            Detection2DArray, '/detections', self._on_detections, 10
        )
        self.get_logger().info('tracker san sang - cho /detections')

    def _on_detections(self, msg: Detection2DArray):
        candidates = [d for d in msg.detections if d.results]
        if not candidates:
            return

        chosen = None
        if self._last_center is not None:
            best_dist = None
            for det in candidates:
                cx = det.bbox.center.position.x
                cy = det.bbox.center.position.y
                dist = math.hypot(cx - self._last_center[0], cy - self._last_center[1])
                if dist <= self._max_jump_px and (best_dist is None or dist < best_dist):
                    best_dist = dist
                    chosen = det

        if chosen is None:
            chosen = max(candidates, key=lambda d: d.results[0].hypothesis.score)

        cx = chosen.bbox.center.position.x
        cy = chosen.bbox.center.position.y
        self._last_center = (cx, cy)

        out = TrackedTarget()
        out.id = 0
        out.center_x = float(cx)
        out.center_y = float(cy)
        out.width = float(chosen.bbox.size_x)
        out.height = float(chosen.bbox.size_y)
        out.confidence = float(chosen.results[0].hypothesis.score)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Tracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
