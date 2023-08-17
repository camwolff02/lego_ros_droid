import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from tag_interface.msg import Tag as TagData
from tag_interface.msg import TagList 

from cv_bridge import CvBridge
import numpy as np
from .aruco_detector import *


PATH_TO_DATA = 'src/tag_detection/tag_detection/data'
DATA_NAME = 'zed_data.npz'

class Tag_Detector(Node):
    
    def __init__(self):
        # ROS2 NODE 
        super().__init__('tag_detector')

        self.subscription = self.create_subscription(Image, 'autonomy/camera_image', self.listener_callback, 10)
        self.bridge = CvBridge()

        self.publisher_ = self.create_publisher(TagList, 'autonomy/tag_data', 10)
        self.cam_publisher_ = self.create_publisher(CompressedImage, 'autonomy/tag_visualization', qos_profile=qos_profile_sensor_data)
        
        # timer_period = 0.01  # seconds
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.detector = Aruco_Detector(DATA_NAME, PATH_TO_DATA, is_stereo=False)       

        self.frame = np.zeros((2, 2))

    def listener_callback(self, image_msg):
        self.frame = self.bridge.imgmsg_to_cv2(image_msg)


    def timer_callback(self):
        if self.frame.shape == (2, 2): return

        tag_list = TagList()
        tag_list.data = []
        
        tags, frame = self.detector.detect_tags(self.frame) 

        for tag in tags:
            tag_msg = TagData()
            tag_msg.id = int(tag.id)
            tag_msg.x_position = tag.x_pos
            tag_msg.y_position = tag.y_pos
            tag_msg.distance = tag.dist
            tag_list.data.append(tag_msg)

        #self.get_logger().info(f"tags found: {len(tag_list.data)}")
        self.publisher_.publish(tag_list)

        if len(tags) > 0:
            self.get_logger().info(f"dist: {tags[0].dist}")
        else:   
            self.get_logger().info(f"searching for tags")


        frame = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_AREA)
        self.cam_publisher_.publish(self.bridge.cv2_to_compressed_imgmsg(frame))



def main(args=None):
    rclpy.init(args=args)
    tag_detector = Tag_Detector()
    rclpy.spin(tag_detector)

    tag_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



