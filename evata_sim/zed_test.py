import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        # ZED kameranın RGB görüntüsünü almak için abone oluyoruz
        self.subscription = self.create_subscription(
            Image,
            '/camera/zed',  # ZED kameranın RGB görüntüsü
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            # Gelen mesajın encoding türünü kontrol et
            if msg.encoding == 'bgr8':  # Eğer RGB görüntü alıyorsanız
                # ROS Image mesajını OpenCV formatına dönüştür
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            elif msg.encoding == 'mono8':  # Eğer siyah-beyaz (grayscale) görüntü alıyorsanız
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            elif msg.encoding == '32FC1':  # Eğer derinlik görüntüsü (depth) alıyorsanız
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            else:
                self.get_logger().warn(f"Unexpected image encoding: {msg.encoding}")
                return
            
            # Görüntüyü ekranda göster
            cv2.imshow("Camera Image", cv_image)
            cv2.waitKey(1)  # OpenCV'nin GUI'sini güncel tutar

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridgeError: {e}')

def main(args=None):
    rclpy.init(args=args)  # ROS 2'yi başlat
    node = ImageSubscriber()  # Node'u oluştur

    try:
        rclpy.spin(node)  # Node'u çalıştır (bu, sürekli olarak gelen verileri dinleyecek)
    except KeyboardInterrupt:
        pass

    node.destroy_node()  # Node'u yok et
    rclpy.shutdown()  # ROS 2'yi kapat
    cv2.destroyAllWindows()  # OpenCV penceresini kapat

if __name__ == '__main__':
    main()
