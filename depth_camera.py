import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ROS(Node):
    def __init__(self):
        super().__init__('demo')
        self._image = None
        self._bridge = CvBridge()
        self.image_subscription = self.create_subscription(Image,'/camera/color/image_raw',self.image_callback,10)
        
        self._depth = None
        self._bridge = CvBridge()
        self.depth_subscription = self.create_subscription(Image,'/camera/depth/image_raw',self.depth_callback,10)

    def image_callback(self, msg):
        self._image = self._bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    def depth_callback(self, msg):
        self._depth = self._bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')

def main(args=None):
    rclpy.init(args=args)
    node = ROS()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            if node._image is None or node._depth is None:
                continue

            image = node._image.copy()
            depth = node._depth.copy()

            # Depth Messages
            height, width = depth.shape
            center_x, center_y = width // 2, height // 2
            depth_at_center = depth[int(center_y), int(center_x)]
            cv2.circle(image, (int(center_x),int(center_y)), 2, (0, 0, 255), -1)
            image = cv2.putText(image, str(depth_at_center),(int(center_x),int(center_y)),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),1,cv2.LINE_AA)

            cv2.imshow("image", image)
            key_code = cv2.waitKey(1)
            if key_code in [27, ord('q')]:
                break

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
