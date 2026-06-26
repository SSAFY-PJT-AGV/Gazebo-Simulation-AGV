import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time


MARKER_INFO = {
    900: "AGV01_START",
    901: "CROSS_ZONE",
    902: "CONVEYOR_ZONE",
}


class ArucoDemoDetector(Node):
    def __init__(self):
        super().__init__('aruco_demo_detector')

        self.image_topic = '/AGV01/agv01_camera/image_raw'
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_1000
        )

        if hasattr(cv2.aruco, 'DetectorParameters'):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(
                self.aruco_dict,
                self.aruco_params
            )
            self.use_new_api = True
        else:
            self.detector = None
            self.use_new_api = False

        self.last_detected_ids = []
        self.last_print_time = 0
        self.print_interval = 1.0

        self.get_logger().info('====================================')
        self.get_logger().info('AGV01 ArUco Demo Detector Started')
        self.get_logger().info(f'Subscribed topic: {self.image_topic}')
        self.get_logger().info('Waiting for marker...')
        self.get_logger().info('====================================')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.use_new_api:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray,
                self.aruco_dict,
                parameters=self.aruco_params
            )

        if ids is None:
            return

        detected_ids = [int(marker_id[0]) for marker_id in ids]

        now = time.time()

        # 같은 마커가 계속 보이면 1초에 한 번만 출력
        if detected_ids == self.last_detected_ids and now - self.last_print_time < self.print_interval:
            return

        self.last_detected_ids = detected_ids
        self.last_print_time = now

        for marker_id in detected_ids:
            location_name = MARKER_INFO.get(marker_id, "UNKNOWN_LOCATION")

            self.get_logger().info('------------------------------------')
            self.get_logger().info(f'[AGV01] Detected marker_{marker_id}')
            self.get_logger().info(f'Location: {location_name}')
            self.get_logger().info('Vision status: MARKER_DETECTED')
            self.get_logger().info('------------------------------------')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDemoDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
