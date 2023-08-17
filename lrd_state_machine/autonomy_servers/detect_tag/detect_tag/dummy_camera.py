import cv2

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from .zed_camera import *


class Dummy_Camera(Node):
    
    def __init__(self, camera=0):
        super().__init__('dummy_camera')
        self.publisher_ = self.create_publisher(Image, 'autonomy/camera_image', 10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # self.cap = cv2.VideoCapture(camera)
        self.zed = ZedCamera(camera)

        self.bridge = CvBridge()
        self.get_logger().info('camera connected')

    def timer_callback(self):
        # ret, frame = self.cap.read()
        ret, frame = self.zed.read()

        if ret:
            self.publisher_.publish(self.bridge.cv2_to_imgmsg(frame))
            self.get_logger().info('Publishing video frame')
        else:
            self.get_logger().info('no camera connected')



def main(args=None):
    rclpy.init(args=args)
    dummy_camera = Dummy_Camera(camera=0)
    rclpy.spin(dummy_camera)

    dummy_camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()