import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class SaveFrame(Node):
    def __init__(self):
        super().__init__('save_camera_frame')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/AGV01/agv01_camera/image_raw',
            self.image_callback,
            10
        )
        self.saved = False
        self.get_logger().info('Waiting for one camera frame...')

    def image_callback(self, msg):
        if self.saved:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        save_path = '/home/ssafy/gazebo_agv_ws/src/agv_sim/scripts/aruco_debug_frame.png'
        cv2.imwrite(save_path, frame)

        self.get_logger().info(f'Saved frame to: {save_path}')
        self.saved = True
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SaveFrame()
    rclpy.spin(node)


if __name__ == '__main__':
    main()