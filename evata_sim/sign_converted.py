#!/usr/bin/env python3.10

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from ament_index_python.packages import get_package_share_directory
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
        package_path = get_package_share_directory('evata_sim')
        model_path = os.path.join(package_path, 'model', 'sol300best.pt')
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.fx = 277.0
        self.tracked_signs = {}
        self.latest_pointcloud = None

        # Kalibrasyon ve sınıf boyutları
        self.distance_calibration_factor = 2.0
        self.real_sizes = {
            'park': 0.4, 'dur': 0.6, '20': 0.3, '30': 0.3,
            'durak': 0.5, 'girisyok': 0.4, 'ilerisag': 0.4,
            'ilerisol': 0.4, 'kirmizi': 0.3, 'parkyasak': 0.4,
            'sag': 0.4, 'sagadonulmez': 0.4, 'sari': 0.3,
            'sol': 0.4, 'soladonulmez': 0.4, 'yesil': 0.3,
            'engellipark': 0.4, 'tasittrafiginekapali': 0.5,
            'yayagecidi': 0.5, 'kavsak': 0.6, 'ikiliyon': 0.4,
            'engellipark_ters': 0.4, 'parkyapilmaz': 0.4
        }

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
                distance = self.calculate_best_distance(x1, y1, x2, y2, class_name)
                self._draw_box(x1, y1, x2, y2, class_name, distance)

            if self.annotated_image.shape[0] > 0 and self.annotated_image.shape[1] > 0:
                cv2.imshow("Tespit Edilen Levhalar", self.annotated_image)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Görüntü Yakalama Hatası: {e}")

    def calculate_best_distance(self, x1, y1, x2, y2, class_name):
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2

        distance = self.calculate_pointcloud_distance(center_x, center_y, x1, y1, x2, y2)

        if distance <= 0:
            distance = self.calculate_distance_from_size(x1, y1, x2, y2, class_name)

        return round(distance * self.distance_calibration_factor, 2)

    def calculate_pointcloud_distance(self, center_x, center_y, x1, y1, x2, y2):
        if self.latest_pointcloud is None:
            return -1

        try:
            sample_points = self.generate_sample_points(x1, y1, x2, y2)
            distances = []

            for u, v in sample_points:
                try:
                    # Koordinatları int'e çeviriyoruz ve sınırları kontrol ediyoruz
                    u = max(0, min(int(u), self.latest_pointcloud.width - 1))
                    v = max(0, min(int(v), self.latest_pointcloud.height - 1))
                    
                    # Point cloud verilerini okuyoruz
                    pc_gen = point_cloud2.read_points(
                        self.latest_pointcloud,
                        field_names=("x", "y", "z"),
                        skip_nans=True,
                        uvs=[(u, v)]
                    )

                    # Generator'dan verileri güvenli bir şekilde alıyoruz
                    for point_data in pc_gen:
                        # Point data'yı liste veya tuple olarak işliyoruz
                        if isinstance(point_data, (list, tuple)) and len(point_data) >= 3:
                            x, y, z = point_data[0], point_data[1], point_data[2]
                        else:
                            continue
                            
                        # NaN ve inf değerlerini kontrol ediyoruz
                        if (not isinstance(x, (int, float)) or 
                            not isinstance(y, (int, float)) or 
                            not isinstance(z, (int, float)) or
                            math.isnan(x) or math.isnan(y) or math.isnan(z) or
                            math.isinf(x) or math.isinf(y) or math.isinf(z)):
                            continue
                            
                        # Z koordinatını (derinlik) mesafe olarak kullanıyoruz
                        dist = abs(float(z))
                        if 0.5 < dist < 20:
                            distances.append(dist)
                            break  # Bu nokta için sadece bir değer alıyoruz
                            
                except Exception as point_error:
                    # Tek bir nokta için hata olursa diğer noktalara geçiyoruz
                    self.get_logger().debug(f"Nokta okuma hatası: {point_error}")
                    continue

            # Mesafe hesaplama
            if len(distances) > 0:
                distances.sort()
                # En yakın 5 mesafeyi alıyoruz (veya tüm mesafeler 5'ten azsa hepsini)
                valid_distances = distances[:min(5, len(distances))]
                return sum(valid_distances) / len(valid_distances)
            else:
                return -1

        except Exception as e:
            self.get_logger().error(f"PointCloud Mesafe Hesaplama Hatası: {e}")
            return -1

    def generate_sample_points(self, x1, y1, x2, y2):
        width = x2 - x1
        height = y2 - y1
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2

        # Sınır kontrolü ekleyerek sample point'leri oluşturuyoruz
        points = [
            (center_x, center_y),
            (x1 + width//4, y1 + height//4),
            (x1 + 3*width//4, y1 + height//4),
            (x1 + width//4, y1 + 3*height//4),
            (x1 + 3*width//4, y1 + 3*height//4),
            (x1, y1),
            (x2, y1),
            (x1, y2),
            (x2, y2)
        ]
        
        # Negatif koordinatları filtreleyelim
        valid_points = [(max(0, x), max(0, y)) for x, y in points if x >= 0 and y >= 0]
        return valid_points

    def calculate_distance_from_size(self, x1, y1, x2, y2, class_name):
        if class_name not in self.real_sizes:
            return -1

        pixel_width = x2 - x1
        real_size = self.real_sizes[class_name]
        focal_length = self.fx

        if pixel_width <= 0:
            return -1

        return (real_size * focal_length) / pixel_width

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