#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge


class CarDetector(Node):

    def __init__(self):
        super().__init__('gimbal_car_detector')

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/drone/gimbal_camera/image_raw',
            self.image_callback,
            10
        )

        # [detected, center_x, center_y, width, height, score]
        self.detection_pub = self.create_publisher(
            Float32MultiArray,
            '/drone/gimbal_car_detection',
            10
        )

        self.debug_pub = self.create_publisher(
            Image,
            '/drone/gimbal_car_debug',
            10
        )

        # Temporary diagnostic view of the thresholded image.
        self.mask_pub = self.create_publisher(
            Image,
            '/drone/gimbal_mask_debug',
            10
        )

        self.get_logger().info('Car detector started')
        self.get_logger().info('Subscribing to /drone/gimbal_camera/image_raw')

    def image_callback(self, msg):

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        height, width = frame.shape[:2]

        # ------------------------------------------------------------
        # 1. Convert to HSV
        # ------------------------------------------------------------
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Prius is gray.
        #
        # Gray:
        #   relatively low saturation
        #   medium/high brightness
        #
        # Yellow lane markings are rejected by low saturation.
        # Very bright white road markings are rejected by V < 230.
        lower = np.array([0, 0, 55])
        upper = np.array([180, 75, 225])

        mask = cv2.inRange(hsv, lower, upper)

        # ------------------------------------------------------------
        # 2. Remove small noise
        # ------------------------------------------------------------
        # Preserve small distant-car details, then reconnect
        # nearby pixels belonging to the same vehicle.
        open_kernel = np.ones((3, 3), np.uint8)
        close_kernel = np.ones((5, 5), np.uint8)

        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, open_kernel
        )

        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE, close_kernel
        )

        # Publish threshold mask for diagnosis.
        try:
            mask_msg = self.bridge.cv2_to_imgmsg(
                mask,
                encoding='mono8'
            )
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)
        except Exception as e:
            self.get_logger().error(f'Mask image conversion failed: {e}')

        # ------------------------------------------------------------
        # 3. Find contours
        # ------------------------------------------------------------
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        best_contour = None
        best_score = 0.0

        image_area = width * height

        for contour in contours:

            area = cv2.contourArea(contour)

            # Reject tiny objects and huge background regions.
            if area < 300:
                continue

            if area > image_area * 0.70:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            # Reject clipped background objects at image boundaries.
            edge_margin = 3
            if (
                x <= edge_margin
                or y <= edge_margin
                or x + w >= width - edge_margin
            ):
                continue

            if w <= 0 or h <= 0:
                continue

            # Reject small fragments and bottom-edge false positives.
            box_center_y = (y + h / 2.0) / float(height)
            if (
                w < width * 0.03
                or h < height * 0.04
                or box_center_y > 0.96
            ):
                continue

            aspect = float(w) / float(h)

            # Prius viewed from above should form a reasonably large
            # elongated/rectangular object.
            if aspect < 0.35 or aspect > 3.2:
                continue

            bbox_area = float(w * h)

            rectangularity = area / bbox_area

            if rectangularity < 0.18:
                continue

            # Large objects get higher scores.
            area_score = min(area / 30000.0, 1.0)

            # Prefer objects with reasonable rectangularity.
            shape_score = min(rectangularity / 0.7, 1.0)

            score = (
                0.60 * area_score +
                0.40 * shape_score
            )

            if score > best_score:
                best_score = score
                best_contour = contour

        # ------------------------------------------------------------
        # 4. Prepare output
        # ------------------------------------------------------------
        detected = 0.0

        center_x = 0.5
        center_y = 0.5
        box_w = 0.0
        box_h = 0.0

        debug = frame.copy()

        if best_contour is not None and best_score > 0.20:

            x, y, w, h = cv2.boundingRect(best_contour)

            cx = x + w / 2.0
            cy = y + h / 2.0

            center_x = cx / float(width)
            center_y = cy / float(height)

            box_w = w / float(width)
            box_h = h / float(height)

            detected = 1.0

            # Draw bounding box.
            cv2.rectangle(
                debug,
                (x, y),
                (x + w, y + h),
                (0, 255, 0),
                3
            )

            # Draw center.
            cv2.circle(
                debug,
                (int(cx), int(cy)),
                8,
                (0, 0, 255),
                -1
            )

            text = (
                f'CAR '
                f'x={center_x:.2f} '
                f'y={center_y:.2f} '
                f'score={best_score:.2f}'
            )

            cv2.putText(
                debug,
                text,
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

        else:

            cv2.putText(
                debug,
                'CAR NOT DETECTED',
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

        # ------------------------------------------------------------
        # 5. Publish detection
        # ------------------------------------------------------------

        detection = Float32MultiArray()

        detection.data = [
            detected,
            float(center_x),
            float(center_y),
            float(box_w),
            float(box_h),
            float(best_score)
        ]

        self.detection_pub.publish(detection)

        # ------------------------------------------------------------
        # 6. Publish debug image
        # ------------------------------------------------------------

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(
                debug,
                encoding='bgr8'
            )

            debug_msg.header = msg.header

            self.debug_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(
                f'Debug image conversion failed: {e}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = CarDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
