import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class StereoPublisher(Node):
    def __init__(self):
        super().__init__('stereo_publisher')
        self.bridge = CvBridge()

        self.pub_left = self.create_publisher(Image, '/left/image_raw', 10)
        self.pub_right = self.create_publisher(Image, '/right/image_raw', 10)

        # otwarcie kamery
        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)

        # Wymuszenie trybu MJPG
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        if not self.cap.isOpened():
            self.get_logger().error("Nie mozna otworzyc urzadzenia /dev/video0")
            return

        # Pętla odświeżająca obraz
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info("Zintegrowana kamera uruchomiona, pobieranie obrazu...")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # Cięcie pobranej klatki w locie na dwie połowy
        mid = frame.shape[1] // 2
        left_img = frame[:, :mid]
        right_img = frame[:, mid:]

        # Przypisanie tego samego znacznika czasu obu klatkom
        ros_time = self.get_clock().now().to_msg()

        msg_left = self.bridge.cv2_to_imgmsg(left_img, "bgr8")
        msg_left.header.stamp = ros_time
        msg_left.header.frame_id = "left_camera_link"

        msg_right = self.bridge.cv2_to_imgmsg(right_img, "bgr8")
        msg_right.header.stamp = ros_time
        msg_right.header.frame_id = "right_camera_link"

        self.pub_left.publish(msg_left)
        self.pub_right.publish(msg_right)


def main(args=None):
    rclpy.init(args=args)
    node = StereoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()


if __name__ == '__main__':
    main()
