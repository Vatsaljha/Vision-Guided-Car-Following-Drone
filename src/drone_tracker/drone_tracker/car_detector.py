#!/usr/bin/env python3

import cv2
import time
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge


class CarDetector(Node):
    """Detect the simulated Prius using its roof ArUco marker only."""

    def __init__(self):
        super().__init__("car_detector")

        self.marker_id = 7
        self.bridge = CvBridge()
        self.last_target_detected = False
        self.last_valid_detection_data = None
        self.last_valid_detection_time = 0.0
        self.detection_grace = 0.18
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self.missed_target_frames = 0

        aruco = cv2.aruco
        self.dictionary = aruco.getPredefinedDictionary(
            aruco.DICT_4X4_50
        )

        if hasattr(aruco, "DetectorParameters"):
            self.parameters = aruco.DetectorParameters()
        else:
            self.parameters = aruco.DetectorParameters_create()

        # Allow smaller markers and improve detection across turns.
        parameter_values = {
            "adaptiveThreshWinSizeMin": 3,
            "adaptiveThreshWinSizeMax": 23,
            "adaptiveThreshWinSizeStep": 10,
            "minMarkerPerimeterRate": 0.006,
            "errorCorrectionRate": 0.7,
        }
        for name, value in parameter_values.items():
            if hasattr(self.parameters, name):
                setattr(self.parameters, name, value)

        if hasattr(self.parameters, "cornerRefinementMethod"):
            self.parameters.cornerRefinementMethod = getattr(
                aruco, "CORNER_REFINE_SUBPIX", 1
            )

        # OpenCV's newer and older ArUco interfaces.
        self.aruco_detector = (
            aruco.ArucoDetector(self.dictionary, self.parameters)
            if hasattr(aruco, "ArucoDetector")
            else None
        )

        self.image_sub = self.create_subscription(
            Image,
            "/drone/camera/image_raw",
            self.image_callback,
            10,
        )

        self.detection_pub = self.create_publisher(
            Float32MultiArray,
            "/drone/car_detection",
            10,
        )

        self.debug_pub = self.create_publisher(
            Image,
            "/drone/car_debug",
            10,
        )

        self.get_logger().info(
            "ArUco car detector started: DICT_4X4_50, target ID 7"
        )

    def _detect_once(self, gray):
        if self.aruco_detector is not None:
            return self.aruco_detector.detectMarkers(gray)

        return cv2.aruco.detectMarkers(
            gray,
            self.dictionary,
            parameters=self.parameters,
        )

    def detect_markers(self, gray):
        # Pass 1: original image (fast path).
        native = self._detect_once(gray)
        corners, ids, rejected = native

        if ids is not None and self.marker_id in ids.flatten():
            self.missed_target_frames = 0
            return native

        # Pass 2: cached CLAHE enhancement for difficult frames.
        enhanced = self.clahe.apply(gray)
        enhanced_result = self._detect_once(enhanced)
        corners, ids, rejected = enhanced_result

        if ids is not None and self.marker_id in ids.flatten():
            self.missed_target_frames = 0
            return enhanced_result

        # Expensive 2x fallback only on every third consecutive miss.
        self.missed_target_frames += 1
        if self.missed_target_frames % 3 != 0:
            return native

        enlarged = cv2.resize(
            enhanced, None, fx=2.0, fy=2.0,
            interpolation=cv2.INTER_CUBIC,
        )
        corners2, ids2, rejected2 = self._detect_once(enlarged)

        if ids2 is not None and self.marker_id in ids2.flatten():
            self.missed_target_frames = 0
            corners2 = [
                np.asarray(corner, dtype=np.float32) / 2.0
                for corner in corners2
            ]
            return corners2, ids2, rejected2

        # Keep the original-frame result for debug display.
        return native

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
        except Exception as exc:
            self.get_logger().error(
                f"Camera image conversion failed: {exc}"
            )
            return

        image_h, image_w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        debug = frame.copy()

        try:
            corners, ids, rejected = self.detect_markers(gray)
        except Exception as exc:
            self.get_logger().error(f"ArUco detection failed: {exc}")
            return

        # Default output means the target marker was not found.
        detected = 0.0
        center_x = 0.5
        center_y = 0.5
        box_w = 0.0
        box_h = 0.0
        size_score = 0.0

        candidates = []

        if ids is not None:
            marker_ids = ids.flatten().astype(int)

            for index, found_id in enumerate(marker_ids):
                points = corners[index].reshape(4, 2)

                # Draw other IDs for debugging, but never follow them.
                if found_id != self.marker_id:
                    points_i = np.round(points).astype(np.int32)
                    cv2.polylines(
                        debug, [points_i], True, (255, 160, 0), 2
                    )
                    cv2.putText(
                        debug,
                        f"ID {found_id}",
                        tuple(points_i[0]),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 160, 0),
                        2,
                    )
                    continue

                side_lengths = [
                    float(np.linalg.norm(points[i] - points[(i + 1) % 4]))
                    for i in range(4)
                ]
                average_side = float(np.mean(side_lengths))
                area = abs(float(cv2.contourArea(points.astype(np.float32))))

                # Very tiny detections are too unreliable for control.
                if average_side < 5.0 or area < 16.0:
                    continue

                candidates.append((area, points, average_side))

        # If ID 7 appears more than once, use its largest image instance.
        if candidates:
            _, points, average_side = max(
                candidates, key=lambda candidate: candidate[0]
            )

            x_min = float(np.clip(points[:, 0].min(), 0, image_w - 1))
            x_max = float(np.clip(points[:, 0].max(), 0, image_w - 1))
            y_min = float(np.clip(points[:, 1].min(), 0, image_h - 1))
            y_max = float(np.clip(points[:, 1].max(), 0, image_h - 1))

            center_x = float(points[:, 0].mean()) / image_w
            center_y = float(points[:, 1].mean()) / image_h
            box_w = max(0.0, x_max - x_min) / image_w
            box_h = max(0.0, y_max - y_min) / image_h
            detected = 1.0

            # Size-based quality indicator, not a calibrated probability.
            size_score = min(1.0, average_side / 80.0)

            polygon = np.round(points).astype(np.int32)
            cv2.polylines(debug, [polygon], True, (0, 255, 0), 3)
            center_px = (
                int(points[:, 0].mean()),
                int(points[:, 1].mean()),
            )
            cv2.circle(debug, center_px, 5, (0, 0, 255), -1)
            cv2.putText(
                debug,
                f"ARUCO ID 7 size={average_side:.0f}px",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
        else:
            cv2.putText(
                debug,
                "ARUCO ID 7 NOT DETECTED",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

        # Suppress single-frame decode flicker without following stale
        # data indefinitely. A fresh valid ID 7 updates this cache.
        now = time.monotonic()
        if detected:
            self.last_valid_detection_time = now
            self.last_valid_detection_data = (
                center_x, center_y, box_w, box_h, size_score
            )
        elif (
            self.last_valid_detection_data is not None
            and now - self.last_valid_detection_time <= self.detection_grace
        ):
            detected = 1.0
            (
                center_x, center_y, box_w, box_h, size_score
            ) = self.last_valid_detection_data

            # Clearly label the brief dropout in the debug view.
            cv2.rectangle(
                debug, (0, 0), (min(image_w, 620), 52), (0, 0, 0), -1
            )
            cv2.putText(
                debug,
                "ID 7 brief dropout - output held",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
            )

        if bool(detected) != self.last_target_detected:
            if detected:
                self.get_logger().info("Roof marker ID 7 acquired")
            else:
                self.get_logger().warn("Roof marker ID 7 lost")
            self.last_target_detected = bool(detected)

        output = Float32MultiArray()
        output.data = [
            detected,
            float(center_x),
            float(center_y),
            float(box_w),
            float(box_h),
            float(size_score),
        ]
        self.detection_pub.publish(output)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)
        except Exception as exc:
            self.get_logger().error(f"Debug image publishing failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = CarDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
