import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

import cv2


class Tag_Visualizer(Node):
    
    def __init__(self):
        super().__init__('tag_visualizer')

        self.subscription = self.create_subscription(CompressedImage, 
                                                     'autonomy/tag_visualization', 
                                                     self.listener_callback, 
                                                     qos_profile=qos_profile_sensor_data, 
                                                     callback_group=ReentrantCallbackGroup())
        self.bridge = CvBridge()

        self.get_logger().info('tag_visualizer node started')


    def listener_callback(self, image_msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(image_msg)
        cv2.imshow('tag orientation visualization', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    tag_visualizer = Tag_Visualizer()
    rclpy.spin(tag_visualizer)

    tag_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()