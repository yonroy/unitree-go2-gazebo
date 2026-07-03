"""Node object_detector: /camera/image -> YOLO (ultralytics, pretrained COCO)
-> /detections (vision_msgs/Detection2DArray).

Chi giu lai detection thuoc 1 trong cac class muc tieu (param target_classes,
list) va confidence >= min_confidence. Da kiem chung thuc te trong Gazebo:
qua cau tron mau cam target_ball (khong texture) KHONG khop on dinh 1 class
COCO duy nhat - YOLO doi qua lai giua "orange" (xa), "sports ball"/"kite"
(gan, bbox lon hon) tuy khoang cach/goc nhin, vi day la vat the trong sim
khong co hoa van that nhu anh huan luyen COCO. Vi vay mac dinh chap nhan ca
3 class nay, threshold thap (0.15) de bam duoc lien tuc hon. Chay o tan so
gioi han (param detection_rate_hz) de khong nghen CPU/GPU.

Phu thuoc ultralytics + torch KHONG cai qua rosdep (khong phai apt package) -
cai thu cong 1 lan:
    python3 -m pip install --user --break-system-packages ultralytics
"""
import time

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

try:
    import torch
    from ultralytics import YOLO
except ImportError as exc:  # pragma: no cover
    torch = None
    YOLO = None
    _IMPORT_ERROR = exc


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        if YOLO is None:
            self.get_logger().error(
                'Khong import duoc ultralytics/torch: '
                f'{_IMPORT_ERROR}. Cai bang: '
                'python3 -m pip install --user --break-system-packages ultralytics'
            )
            raise SystemExit(1)

        self.declare_parameter('target_classes', ['orange', 'sports ball', 'kite', 'frisbee'])
        self.declare_parameter('min_confidence', 0.10)
        self.declare_parameter('detection_rate_hz', 10.0)
        self.declare_parameter('model_name', 'yolov8n.pt')

        self.target_classes = set(self.get_parameter('target_classes').value)
        self.min_confidence = self.get_parameter('min_confidence').value
        self._min_period_s = 1.0 / self.get_parameter('detection_rate_hz').value
        self._last_run_time = 0.0

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Nap YOLO model, device={self.device}')
        self.model = YOLO(self.get_parameter('model_name').value)

        self.bridge = CvBridge()
        self.det_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self.image_sub = self.create_subscription(Image, '/camera/image', self._on_image, 1)

        self.get_logger().info(
            f'object_detector san sang - target_classes={sorted(self.target_classes)}, '
            f'min_confidence={self.min_confidence}'
        )

    def _on_image(self, msg: Image):
        now = time.monotonic()
        if now - self._last_run_time < self._min_period_s:
            return
        self._last_run_time = now

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # PHAI truyen conf=min_confidence: mac dinh ultralytics loc o nguong 0.25,
        # se cat het detection duoi 0.25 TRUOC khi minh loc theo min_confidence
        # (bug da tung lam /detections rong dù muc tieu van thay - da kiem chung).
        results = self.model.predict(
            frame, device=self.device, conf=self.min_confidence, verbose=False
        )[0]
        names = results.names

        out = Detection2DArray()
        out.header = msg.header

        for box in results.boxes:
            cls_id = int(box.cls[0])
            cls_name = names.get(cls_id, str(cls_id))
            confidence = float(box.conf[0])
            if cls_name not in self.target_classes or confidence < self.min_confidence:
                continue

            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]

            det = Detection2D()
            det.header = msg.header
            det.bbox = BoundingBox2D()
            det.bbox.center.position.x = (x1 + x2) / 2.0
            det.bbox.center.position.y = (y1 + y2) / 2.0
            det.bbox.size_x = x2 - x1
            det.bbox.size_y = y2 - y1

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = cls_name
            hyp.hypothesis.score = confidence
            det.results.append(hyp)

            out.detections.append(det)

        self.det_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
