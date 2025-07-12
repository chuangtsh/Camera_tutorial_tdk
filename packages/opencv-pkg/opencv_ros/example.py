import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class IMG(Node):
    def __init__(self):
        super().__init__('example_node')
        self.bridge = CvBridge()
        self.img_sub = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.image_callback,
            10
        )
        self.img_publisher = self.create_publisher(Image, 'processed_image', 10)

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().info('Received an image!')
            self.publish_processed_image()
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def publish_processed_image(self):
        try:
            # 創一個篩選出橘色的mask
            lower_bound = np.array([0, 50, 50]) # 三個參數分別對應：色相, 飽和度, 亮度
            upper_bound = np.array([30, 255, 255])
            mask = cv2.inRange(self.cv_image, lower_bound, upper_bound)
            orange_img = cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask)
            gray_img = cv2.cvtColor(orange_img, cv2.COLOR_BGR2GRAY)
            _, binary_img = cv2.threshold(gray_img, 100, 255, cv2.THRESH_BINARY) 
            contours, _ = cv2.findContours(binary_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            centers = []
            # 計算每個輪廓的中心點(質心)
            for cnt in contours:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    centers.append((cx, cy))
            density_map = np.zeros((self.cv_image.shape[0], self.cv_image.shape[1]), dtype=np.uint8)
            if len(centers) > 5:
                for (x, y) in centers:
                    cv2.circle(density_map, (x, y), radius=10, color=255, thickness=-1)

                # 模糊處理 + 二值化找高密度區
                density_map = cv2.GaussianBlur(density_map, (25, 25), 0)
                _, hot = cv2.threshold(density_map, 127, 255, cv2.THRESH_BINARY)

                # 找出高密度區的輪廓並畫出最大區域的方框
                regions, _ = cv2.findContours(hot, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if regions:
                    max_region = max(regions, key=cv2.contourArea)
                    x, y, w, h = cv2.boundingRect(max_region)
                    cv2.rectangle(self.cv_image, (x, y), (x+w, y+h), (0, 0, 255), 2)
                    # 計算方框中心點
                    cx = x + w // 2
                    cy = y + h // 2

                    # 在圖上標記中心點
                    cv2.circle(self.cv_image, (cx, cy), 5, (0, 255, 0), -1)
                    cv2.putText(self.cv_image, f"({cx},{cy})", (cx + 10, cy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)
            msg = self.bridge.cv2_to_imgmsg(self.cv_image, encoding='8UC3')
            self.img_publisher.publish(msg)
            self.get_logger().info('Processed image published!')
        except Exception as e:
            self.get_logger().error(f'Error publishing processed image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = IMG()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()