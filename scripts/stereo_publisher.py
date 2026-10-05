#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class StereoPublisher(Node):
    def __init__(self):
        super().__init__('stereo_publisher')
        self.bridge = CvBridge()


        self.sub_image = self.create_subscription(
            Image, 
            '/image_raw', 
            self.image_callback, 
            10
        )

        #publikatory dla lewego i prawego oka
        self.pub_left = self.create_publisher(Image, '/left/image_raw', 10)
        self.pub_right = self.create_publisher(Image, '/right/image_raw', 10)

        self.get_logger().info("Węzeł tnący obraz uruchomiony. Czekam na /image_raw...")

    def image_callback(self, msg):
        try:

            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Błąd konwersji obrazu: {e}")
            return


        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)


        mid = gray_image.shape[1] // 2
        left_img = gray_image[:, :mid]
        right_img = gray_image[:, mid:]

        try:

            msg_left = self.bridge.cv2_to_imgmsg(left_img, "mono8")
            msg_left.header.stamp = msg.header.stamp
            msg_left.header.frame_id = "left_camera_link"

            msg_right = self.bridge.cv2_to_imgmsg(right_img, "mono8")
            msg_right.header.stamp = msg.header.stamp  
            msg_right.header.frame_id = "right_camera_link"


            self.pub_left.publish(msg_left)
            self.pub_right.publish(msg_right)
        except Exception as e:
            self.get_logger().error(f"Błąd publikacji: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = StereoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
