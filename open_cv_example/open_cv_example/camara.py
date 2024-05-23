import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
import cv2
import numpy as np
from std_msgs.msg import Float32

class VideoDecoder(Node):
    def __init__(self):
        super().__init__('video_decoder')
        self.get_logger().info('Initializing VideoDecoder Node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(CompressedImage, 'image', self.camera_callback, 10)
        self.get_logger().info('Subscription to /image topic created')
        self.publisher = self.create_publisher(Image, '/processed_img', 10)
        self.XL_publisher = self.create_publisher(Float32, '/puzzlebot_line', 10)
        self.Camera_Center_publisher = self.create_publisher(Float32, '/camera_center', 10)
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
            else:
                self.get_logger().error('Failed to decode image')
        except Exception as e:
            self.get_logger().error(f'Error in decode_and_display: {e}')

    def timer_callback(self):
        if self.cv_image is not None:
            try:
                resized_img = cv2.resize(self.cv_image, (480, 360))
                processed_frame, promXL, centerX = self.detect_line(resized_img)
                cv2.imshow('Video Stream', processed_frame)
                cv2.waitKey(1)
                if processed_frame.ndim == 2:  # Check if the image is grayscale
                    self.publisher.publish(self.bridge.cv2_to_imgmsg(processed_frame, encoding='mono8'))
                else:
                    self.publisher.publish(self.bridge.cv2_to_imgmsg(processed_frame, encoding='bgr8'))
                self.get_logger().info('Image Published')
                self.XL_publisher.publish(Float32(data=promXL))
                print('Posicion Linea:', promXL)
                self.Camera_Center_publisher.publish(Float32(data=centerX))
            except Exception as e:
                self.get_logger().error(f'Error in publishing image: {e}')
        else:
            self.get_logger().info('No image available to publish')
    
    def detect_line(self, frame):
        try:
            # Get and show the dimensions of original image
            OriginalHeight, OriginalWidth = frame.shape[:2]
            HalfWidth = OriginalWidth / 2

            # Convert to grayscale and apply blur
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # blur = cv2.medianBlur(gray, 9)

            # Crop image
            cropped_image = gray[OriginalHeight-60:OriginalHeight, 20:OriginalWidth-20]

            # Apply Canny edge detection
            canny = cv2.Canny(cropped_image, 20, 30, apertureSize=3)

            # Detect lines using Hough transform
            lines = cv2.HoughLines(canny, 1, np.pi/180, 25)

            arrL = np.array([], dtype=np.float64)

            if lines is not None:
                for r_theta in lines:
                    arr = np.array(r_theta[0], dtype=np.float64)
                    r, theta = arr
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * r
                    y0 = b * r
                    x1 = int(x0 + 1000 * (-b))
                    y1 = int(y0 + 1000 * a)
                    x2 = int(x0 - 1000 * (-b))
                    y2 = int(y0 - 1000 * a)
                    cv2.line(cropped_image, (x1, y1), (x2, y2), (10, 10, 10), 5)
                    xl = (x1 + x2) / 2
                    arrL = np.append(arrL, xl)

                promXL = np.mean(arrL)
            else:
                promXL = HalfWidth

            return cropped_image, promXL, HalfWidth
        except Exception as e:
            self.get_logger().error(f'Error in detect_line: {e}')
            return frame, 0, 0

def main(args=None):
    rclpy.init(args=args)
    video_decoder = VideoDecoder()
    rclpy.spin(video_decoder)
    video_decoder.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
