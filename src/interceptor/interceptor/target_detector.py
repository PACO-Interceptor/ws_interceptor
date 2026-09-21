"""ROS 2 node for YOLO-based target detection and camera bearing estimation."""

import time
from typing import Optional

from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3Stamped
from interceptor.bearing import pixel_to_bearing, subtended_angle
from interceptor_msgs.msg import TargetSighting
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class TargetDetector(Node):
    """Detects targets in camera frames and publishes detections and bearing."""

    def __init__(self) -> None:
        """Initialize parameters, publishers, subscriptions, and YOLO model."""
        super().__init__('target_detector')

        self.declare_parameter('image_topic', '/interceptor/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/interceptor/camera/camera_info')
        self.declare_parameter('model_path', 'yolo26n.pt')
        self.declare_parameter('target_class', '')
        self.declare_parameter('confidence', 0.25)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('max_rate_hz', 10.0)
        self.declare_parameter('publish_annotated', True)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter(
            'camera_info_topic'
        ).get_parameter_value().string_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        # Una o varias clases separadas por comas; vacio = cualquier clase.
        target_class = self.get_parameter('target_class').get_parameter_value().string_value
        self._target_classes = {c.strip() for c in target_class.split(',') if c.strip()}
        self._confidence = self.get_parameter(
            'confidence'
        ).get_parameter_value().double_value
        self._imgsz = self.get_parameter('imgsz').get_parameter_value().integer_value
        self._device = self.get_parameter('device').get_parameter_value().string_value
        self._max_rate_hz = self.get_parameter(
            'max_rate_hz'
        ).get_parameter_value().double_value
        self._publish_annotated = self.get_parameter(
            'publish_annotated'
        ).get_parameter_value().bool_value

        self._cv_bridge = CvBridge()
        self._camera_info: Optional[CameraInfo] = None
        self._last_inference_time: Optional[float] = None

        self._model = None
        try:
            from ultralytics import YOLO
            self._model = YOLO(model_path)
        except Exception as err:
            self.get_logger().error(f'Failed to load YOLO model "{model_path}": {err}')
            self._model = None

        self._detections_pub = self.create_publisher(
            Detection2DArray,
            'interceptor/detections',
            10,
        )
        self._bearing_pub = self.create_publisher(
            Vector3Stamped,
            'interceptor/target_bearing',
            10,
        )
        # Bearing y tamano angular juntos: es lo que consume el guiado visual.
        self._sighting_pub = self.create_publisher(
            TargetSighting,
            'interceptor/target_sighting',
            10,
        )
        self._annotated_image_pub = self.create_publisher(
            Image,
            'interceptor/detections/image',
            10,
        )

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._camera_info_callback,
            qos_sensor,
        )

        self._image_sub = self.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            qos_sensor,
        )

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """Store the latest camera intrinsic parameters."""
        self._camera_info = msg

    def _image_callback(self, msg: Image) -> None:
        """Process incoming camera image and publish detections."""
        if self._model is None:
            return

        if self._max_rate_hz > 0.0:
            now = time.monotonic()
            if self._last_inference_time is not None:
                if (now - self._last_inference_time) < (1.0 / self._max_rate_hz):
                    return
            self._last_inference_time = now

        try:
            cv_image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as err:
            self.get_logger().warning(f'cv_bridge conversion failed: {err}')
            return

        try:
            results = self._model(
                cv_image,
                conf=self._confidence,
                imgsz=self._imgsz,
                device=self._device,
                verbose=False,
            )
        except Exception as err:
            self.get_logger().warning(f'Inference failed: {err}')
            return

        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        best_target_center = None
        best_target_width = 0.0
        best_target_class = ''
        highest_score = -1.0

        if results and len(results) > 0:
            result = results[0]
            boxes = result.boxes
            if boxes is not None and len(boxes) > 0:
                xywh_data = boxes.xywh.cpu().numpy()
                conf_data = boxes.conf.cpu().numpy()
                cls_data = boxes.cls.cpu().numpy()

                for i in range(len(boxes)):
                    cx_box, cy_box, w_box, h_box = xywh_data[i]
                    score = float(conf_data[i])
                    cls_id = int(cls_data[i])
                    class_name = str(self._model.names.get(cls_id, cls_id))

                    detection = Detection2D()
                    detection.header = msg.header
                    detection.bbox.center.position.x = float(cx_box)
                    detection.bbox.center.position.y = float(cy_box)
                    detection.bbox.size_x = float(w_box)
                    detection.bbox.size_y = float(h_box)

                    hypothesis_with_pose = ObjectHypothesisWithPose()
                    hypothesis_with_pose.hypothesis.class_id = class_name
                    hypothesis_with_pose.hypothesis.score = score
                    detection.results.append(hypothesis_with_pose)

                    detections_msg.detections.append(detection)

                    if not self._target_classes or class_name in self._target_classes:
                        if score > highest_score:
                            highest_score = score
                            best_target_center = (float(cx_box), float(cy_box))
                            best_target_width = float(w_box)
                            best_target_class = class_name

        self._detections_pub.publish(detections_msg)

        if best_target_center is not None and self._camera_info is not None:
            k = self._camera_info.k
            fx = float(k[0])
            fy = float(k[4])
            cx = float(k[2])
            cy = float(k[5])
            if fx != 0.0 and fy != 0.0:
                try:
                    bx, by, bz = pixel_to_bearing(
                        best_target_center[0],
                        best_target_center[1],
                        fx,
                        fy,
                        cx,
                        cy,
                    )
                    bearing_msg = Vector3Stamped()
                    bearing_msg.header.stamp = msg.header.stamp
                    bearing_msg.header.frame_id = 'interceptor/camera_link'
                    bearing_msg.vector.x = bx
                    bearing_msg.vector.y = by
                    bearing_msg.vector.z = bz
                    self._bearing_pub.publish(bearing_msg)

                    # El angulo es opcional: si no sale, el guiado se queda con
                    # el bearing y pierde solo el tiempo hasta impacto.
                    try:
                        theta = subtended_angle(
                            best_target_center[0],
                            best_target_center[1],
                            best_target_width,
                            fx,
                            cx,
                            cy,
                        )
                    except ValueError:
                        theta = 0.0

                    sighting_msg = TargetSighting()
                    sighting_msg.header.stamp = msg.header.stamp
                    sighting_msg.header.frame_id = 'interceptor/camera_link'
                    sighting_msg.bearing.x = bx
                    sighting_msg.bearing.y = by
                    sighting_msg.bearing.z = bz
                    sighting_msg.subtended_angle = theta
                    sighting_msg.confidence = highest_score
                    sighting_msg.class_id = best_target_class
                    self._sighting_pub.publish(sighting_msg)
                except ValueError:
                    pass

        if self._publish_annotated and results and len(results) > 0:
            annotated_frame = results[0].plot()
            annotated_msg = self._cv_bridge.cv2_to_imgmsg(
                annotated_frame,
                encoding='bgr8',
            )
            annotated_msg.header = msg.header
            self._annotated_image_pub.publish(annotated_msg)


def main(args=None):
    """Execute the target_detector node."""
    rclpy.init(args=args)
    node = TargetDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
