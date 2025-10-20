#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import os
from ultralytics import YOLO

class VisionDetector(Node):
    def __init__(self):
        super().__init__('vision_detector')
        self.sub = self.create_subscription(Image, '/carla/hero/rgb_front/image', self.image_cb, 10)
        self.flag_pub = self.create_publisher(Bool, '/carla/hero/stop_flag', 10)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.save_dir = os.path.join(os.getcwd(), 'saved_images')
        os.makedirs(self.save_dir, exist_ok=True)
        self.image_count = 0
        self.target_class = 'stop sign'

    def image_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model.predict(img, conf=0.75)
        found = False
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                name = self.model.names[cls_id]
                cv2.rectangle(img, (x1,y1), (x2,y2), (0,255,0), 2)
                cv2.putText(img, f"{name}:{conf:.2f}", (x1, max(0,y1-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                if name == self.target_class and conf > 0.75:
                    found = True
        if found:
            path = os.path.join(self.save_dir, f'image_{self.image_count:05d}.jpg')
            cv2.imwrite(path, img)
            self.image_count += 1
            msg_flag = Bool()
            msg_flag.data = True
            self.flag_pub.publish(msg_flag)
            self.get_logger().info('Stop sign detected and flag published')


def main(args=None):
    rclpy.init(args=args)
    node = VisionDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
