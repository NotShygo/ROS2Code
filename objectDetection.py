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

            for box, cls_idx, conf_val in zip(boxes.xyxy, boxes.cls, boxes.conf):
                a = box.tolist()
                first_point = (int(a[0]), int(a[1]))
                last_point = (int(a[2]), int(a[3]))
                cv2.rectangle(image, first_point, last_point, (255, 255, 0), 1)

                cls = int(cls_idx) # 使用當前迭代的類別索引
                conf = "{:.2f}".format(conf_val.item()) # 使用當前迭代的信心度

                org = [int(a[0]), int(a[1]) - 5] # 稍微往上一點，才不會壓到框
                text = names[cls] + " " + conf
                cv2.putText(image, text, tuple(org), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 0), 2)
            
            cv2.imshow("image", image)
            if cv2.waitKey(1) == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
