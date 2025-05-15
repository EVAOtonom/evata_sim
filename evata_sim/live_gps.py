import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import math
from ament_index_python.packages import get_package_share_directory

class GPSCorrector(Node):
    def __init__(self):
        super().__init__('gps_from_map')
        self.subscription = self.create_subscription(Odometry, '/odom', self.callback, 10)
        self.publisher = self.create_publisher(NavSatFix, '/gps_corrected', 10)
        self.heading_pub = self.create_publisher(Float32, '/gps/heading', 10)
        self.ref_points = self.load_map()
        self.prev_x = None
        self.prev_y = None

    def load_map(self):
        package_path = get_package_share_directory('evata_sim')
        path = os.path.join(package_path, 'gps_data', 'gps_map.txt')
        path = os.path.abspath(path)
        ref_points = []
        with open(path, 'r') as f:
            for line in f:
                if line.strip().startswith("#") or len(line.strip()) == 0:
                    continue
                x, y, lat, lon = line.strip().split()
                ref_points.append((float(x), float(y), float(lat), float(lon)))
        return ref_points

    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # GPS düzeltme
        nearest = sorted(self.ref_points, key=lambda p: (p[0]-x)**2 + (p[1]-y)**2)[:3]
        total_weight = 0.0
        lat_sum = 0.0
        lon_sum = 0.0
        for px, py, plat, plon in nearest:
            dist = math.hypot(px - x, py - y) + 0.001
            weight = 1.0 / dist
            total_weight += weight
            lat_sum += plat * weight
            lon_sum += plon * weight
        avg_lat = lat_sum / total_weight
        avg_lon = lon_sum / total_weight

        # Yayınla: düzeltilmiş GPS
        msg_out = NavSatFix()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.header.frame_id = "gps_corrected"
        msg_out.latitude = avg_lat
        msg_out.longitude = avg_lon
        msg_out.altitude = 0.0
        self.publisher.publish(msg_out)

        # Heading hesabı (araç hareket yönü)
        if self.prev_x is not None and self.prev_y is not None:
            dx = x - self.prev_x
            dy = y - self.prev_y
            if abs(dx) > 0.001 or abs(dy) > 0.001:
                heading_rad = math.atan2(dy, dx)
                self.last_heading = (math.degrees(heading_rad) + 360.0) % 360.0

        # Her zaman yayınla (son heading varsa)
        if hasattr(self, 'last_heading'):
            self.heading_pub.publish(Float32(data=self.last_heading))
            self.get_logger().info(f"Heading: {self.last_heading:.2f}°")


        self.prev_x = x
        self.prev_y = y

        self.get_logger().info(f"Corrected GPS: lat={avg_lat:.7f}, lon={avg_lon:.7f}")

def main(args=None):
    rclpy.init(args=args)
    node = GPSCorrector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
