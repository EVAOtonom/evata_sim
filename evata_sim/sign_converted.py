#!/usr/bin/env python3.10

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import logging
from sensor_msgs_py import point_cloud2
import math

logging.getLogger('ultralytics').setLevel(logging.ERROR)

class SignDetector(Node):
    def __init__(self):
        super().__init__('sign_detector_node')
        self.model = YOLO("/home/otonom/ros2_ws/src/evata_sim/evata_sim/train16best.pt")
        self.bridge = CvBridge()
        self.fx = 277.0
        self.tracked_signs = {}
        self.latest_pointcloud = None
        
        # Subscribers
        self.create_subscription(Image, "/depth_camera/zed/image", self.color_image_callback, 10)
        self.create_subscription(CameraInfo, "/depth_camera/zed/camera_info", self.camera_info_callback, 10)
        self.create_subscription(PointCloud2, "/depth_camera/zed/points", self.point_cloud_callback, 10)
    
    def camera_info_callback(self, msg):
        self.fx = msg.k[0]

    def point_cloud_callback(self, msg):
        self.latest_pointcloud = msg

    def color_image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            results = self.model(cv_image)
            self.annotated_image = cv_image.copy()
            
            detected_signs = {}

            for r in results:
                for box in r.boxes:
                    if box.conf > 0.7:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        class_name = self.model.names[int(box.cls)]
                        detected_signs[class_name] = (x1, y1, x2, y2, 0)

            updated_tracked_signs = {}

            for class_name, (x1, y1, x2, y2, frame_count) in self.tracked_signs.items():
                if class_name in detected_signs:
                    updated_tracked_signs[class_name] = detected_signs[class_name]
                elif frame_count < 5:
                    updated_tracked_signs[class_name] = (x1, y1, x2, y2, frame_count + 1)

            for class_name, values in detected_signs.items():
                if class_name not in updated_tracked_signs:
                    updated_tracked_signs[class_name] = values

            self.tracked_signs = updated_tracked_signs

            for class_name, (x1, y1, x2, y2, _) in self.tracked_signs.items():
                distance = self.calculate_distance_pointcloud((x1 + x2) // 2, (y1 + y2) // 2)

                if distance == -1:
                    distance = self.calculate_distance(x1, y1, x2, y2, class_name)

                self._draw_box(x1, y1, x2, y2, class_name, distance)

            if self.annotated_image.shape[0] > 0 and self.annotated_image.shape[1] > 0:
                cv2.imshow("Tespit Edilen Levhalar", self.annotated_image)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Görüntü Yakalama Hatası: {e}")

    def calculate_distance_pointcloud(self, center_x, center_y):
        if self.latest_pointcloud is None:
            return -1

        try:
            gen = point_cloud2.read_points(self.latest_pointcloud, field_names=("x", "y", "z"), skip_nans=True)
            valid_points = [(px, py, pz) for px, py, pz in gen if not math.isinf(px) and not math.isnan(px)]
            
            if not valid_points:
                return -1

            distances = [pz for px, py, pz in valid_points if abs(px - center_x) < 5 and abs(py - center_y) < 5]

            return round(sum(distances) / len(distances), 2) if distances else -1

        except Exception as e:
            self.get_logger().error(f"PointCloud Mesafe Hesaplama Hatası: {e}")
            return -1

    def calculate_distance(self, x1, y1, x2, y2, class_name):
        real_width = 0.5  
        bbox_width = max(x2 - x1, 1)

        estimated_distance = (real_width * self.fx) / bbox_width
        return estimated_distance * 1.70

    def _draw_box(self, x1, y1, x2, y2, class_name, distance):
        cv2.rectangle(self.annotated_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.putText(self.annotated_image, f"{class_name}: {distance:.2f}m", 
                    (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

def main(args=None):
    rclpy.init(args=args)
    node = SignDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Başarılı Kod Durdurma.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
