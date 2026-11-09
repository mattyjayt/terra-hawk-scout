"""PersonDetector Node."""

import time

import cv2

from cv_bridge import CvBridge

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from ultralytics import YOLO

from vision_msgs.msg import (Detection2D,
                             Detection2DArray, ObjectHypothesisWithPose)


class PersonDetector(Node):
    """PersonDetector Node."""

    def __init__(self):
        """Start Person Detector Node."""
        super().__init__('person_detector')
        self.declare_parameter('model_path', '')
        self.declare_parameter('imgsz', 320)
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('use_tracking', True)
        self.declare_parameter('debug_image', False)
        self.declare_parameter('debug_rate', 5.0)
        model_path = self.get_parameter('model_path').value
        if not model_path:
            raise RuntimeError('set -p model_path:=<folder>_ncnn_model')
        self._imgsz = self.get_parameter('imgsz').value
        self._conf = self.get_parameter('conf_threshold').value
        self._use_tracking = self.get_parameter('use_tracking').value
        self._debug = self.get_parameter('debug_image').value
        self._debug_period = 1.0 / self.get_parameter('debug_rate').value
        self._bridge = CvBridge()
        self._model = YOLO(model_path, task='detect')
        self._last_debug = 0.0
        self._frames = 0
        self._infer_ms = 0.0
        image_qos = QoSProfile(depth=1,
                               reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(Detection2DArray,
                                          'detections',
                                          qos_profile_sensor_data)
        self._pub_debug = self.create_publisher(
            CompressedImage, 'detections/image/compressed',
            qos_profile_sensor_data)
        self.create_subscription(Image, 'camera/image_raw',
                                 self._on_image, image_qos)
        self.create_timer(1.0, self._log_rate)
        self.get_logger().info(f'model {model_path}, imgsz {self._imgsz}')

    def _on_image(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        t0 = time.perf_counter()
        if self._use_tracking:
            results = self._model.track(frame, persist=True, classes=[0],
                                        conf=self._conf, imgsz=self._imgsz,
                                        tracker='bytetrack.yaml',
                                        verbose=False)
        else:
            results = self._model.predict(frame, classes=[0], conf=self._conf,
                                          imgsz=self._imgsz, verbose=False)
        self._infer_ms = (time.perf_counter() - t0) * 1000.0
        self._pub.publish(self._to_msg(results[0], msg.header))

        if self._debug:
            self._publish_debug(results[0], msg.header)
        self._frames += 1

    def _to_msg(self, result, header) -> Detection2DArray:
        out = Detection2DArray()
        out.header = header

        for box in result.boxes:
            cx, cy, w, h, = box.xywh[0].tolist()
            d = Detection2D()
            d.bbox.center.position.x = cx
            d.bbox.center.position.y = cy
            d.bbox.center.theta = 0.0
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = 'person'
            hyp.hypothesis.score = float(box.conf[0])
            d.results.append(hyp)
            d.id = '' if box.id is None else str(int(box.id[0]))
            out.detections.append(d)

        return out

    def _publish_debug(self, result, header) -> None:
        now = time.monotonic()
        if now - self._last_debug < self._debug_period:
            return

        self._last_debug = now
        ok, buf = cv2.imencode('.jpg', result.plot(),
                               [cv2.IMWRITE_JPEG_QUALITY, 70])

        if ok:
            self._pub_debug.publish(CompressedImage(header=header,
                                                    format='jpeg',
                                                    data=buf.tobytes()))

    def _log_rate(self) -> None:
        self.get_logger().info(
            f'{self._frames} frames/s, inference {self._infer_ms:.0f} ms')
        self._frames = 0


def main(args=None):
    """Start the PersonDetector Node."""
    rclpy.init(args=args)
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
