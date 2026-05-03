import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

_image = None
_bridge = CvBridge()

def image_callback(msg):
    global _image
    _image = _bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('demo')

    # Subscribers
    node.create_subscription(Image, '/image_raw', image_callback, 10)

    model = YOLO("yolo26n.pt")
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node)

            if _image is None:
                print("No Image!")
                continue

            image = _image.copy()

            result = model(image,verbose=False)
            boxes = result[0].boxes
            names = result[0].names

            for i in boxes.xyxy:
                a = i.tolist()
                first_point = (int(a[0]), int(a[1]))
                last_point = (int(a[2]), int(a[3]))
                cv2.rectangle(image, first_point, last_point, (0, 255, 0), 2)

                cls = int(boxes.cls[0])
                conf = boxes.conf[0].item()
                conf = "{:.2f}".format(conf)
                

                org = [int(a[0]), int(a[1])]
                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 1
                color = (0, 255, 0)
                thickness = 2

                text = names[cls]+ " "+conf
                cv2.putText(image, text, org, font, fontScale, color, thickness)
            
            cv2.imshow("image", image)
            if cv2.waitKey(1) == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()