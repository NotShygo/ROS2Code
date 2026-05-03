import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

# ------------------------------
# Global variables
# ------------------------------
_image = None
_depth = None
_bridge = CvBridge()
_publisher = None          # will be assigned in main

# ------------------------------
# Callback functions
# ------------------------------
def image_callback(msg):
    global _image
    _image = _bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

def depth_callback(msg):
    global _depth
    _depth = _bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

def msg_publish(msg):
    """Publish a string message."""
    global _publisher
    ros_msg = String()
    ros_msg.data = msg
    _publisher.publish(ros_msg)

# ------------------------------
# Main routine
# ------------------------------
if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('demo')

    # Publisher
    _publisher = node.create_publisher(String, 'pymsg', 10)

    # Subscribers
    node.create_subscription(Image, '/camera/color/image_raw', image_callback, 10)
    node.create_subscription(Image, '/camera/depth/image_raw', depth_callback, 10)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)

            if _image is None or _depth is None:
                continue

            image = _image.copy()
            depth = _depth.copy()

            height, width = depth.shape
            center_x, center_y = width // 2, height // 2
            depth_at_center = depth[center_y, center_x]

            # Draw circle and text
            cv2.circle(image, (center_x, center_y), 2, (0, 0, 255), -1)
            image = cv2.putText(image, str(depth_at_center),
                                (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX,
                                1, (0, 0, 255), 1, cv2.LINE_AA)

            cv2.imshow("image", image)
            key_code = cv2.waitKey(1)

            if key_code in [27, ord('q')]:
                break
            elif key_code == ord('h'):
                msg_publish("Hi")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
