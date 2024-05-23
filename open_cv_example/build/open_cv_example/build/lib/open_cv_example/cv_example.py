import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
import cv2
import numpy as np

class VideoDecoder(Node):
    def __init__(self):
        super().__init__('video_decoder')
        self.get_logger().info('Initializing VideoDecoder Node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(CompressedImage, 'image', self.camera_callback, 10)
        self.get_logger().info('Subscription to /image topic created')
        self.publisher = self.create_publisher(Image, '/decompressed_img', 10)
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.cv_image = None  # Initialize cv_image to None

    def camera_callback(self, msg):
        self.get_logger().info('Received a CompressedImage message')
        
        try:
            # Convert the ROS CompressedImage message to a numpy array
            np_arr = np.asarray(bytearray(msg.data), dtype=np.uint8)
            self.get_logger().info(f'Image data converted to numpy array of shape: {np_arr.shape}')
            
            # Decode the numpy array into an OpenCV image
            self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if self.cv_image is not None:
                self.get_logger().info('Image successfully decoded')
                # Display the image
                cv2.imshow('Video Stream', self.cv_image)
                cv2.waitKey(1)
            else:
                self.get_logger().error('Failed to decode image')
        except Exception as e:
            self.get_logger().error(f'Error in decode_and_display: {e}')

    def timer_callback(self):
        if self.cv_image is not None:
            try:
                resized_img = cv2.resize(self.cv_image, (480, 360))
                self.publisher.publish(self.bridge.cv2_to_imgmsg(resized_img, encoding='bgr8'))
                self.get_logger().info('Image Published')
            except Exception as e:
                self.get_logger().error(f'Error in publishing image: {e}')
        else:
            self.get_logger().info('No image available to publish')

def main(args=None):
    rclpy.init(args=args)
    video_decoder = VideoDecoder()
    rclpy.spin(video_decoder)
    video_decoder.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
