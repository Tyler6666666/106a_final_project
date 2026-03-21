import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point   

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.error_pub = self.create_publisher(Point, '/aruco/error', 10)
        # subscribe to the drone's camera topic
        self.subscription = self.create_subscription(
            Image,
            '/drone1/image_raw',
            self.image_callback,
            qos_profile_sensor_data)
        self.bridge = CvBridge()
        
        # load ArUco 
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        
        self.get_logger().info('ArucoDetector initialized and ready to detect markers!')

    def image_callback(self, msg):
        try:
            # 1. convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Debug Window", cv_image)
            cv2.waitKey(1)
            # 2. detect ArUco markers in the image
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
            
            # 3. if markers are detected, draw them on the image and log the IDs
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                self.get_logger().info(f"target found! ArUco ID: {ids.flatten()}")
                c = corners[0][0]
                center_x = (c[0][0] + c[2][0]) / 2
                center_y = (c[0][1] + c[2][1]) / 2
            
            # 粗略计算面积（用宽*高）
                width = c[1][0] - c[0][0]
                height = c[3][1] - c[0][1]
                area = width * height
            
            # 画面中心
                img_center_x = cv_image.shape[1] / 2
                img_center_y = cv_image.shape[0] / 2
            
            # 计算误差
                error_msg = Point()
                error_msg.x = float(center_x - img_center_x)
                error_msg.y = float(center_y - img_center_y)
                error_msg.z = float(area) # 用 z 轴传递面积信息
            
                self.error_pub.publish(error_msg)
            else:
                # Publish a zero error message when target is not found
                error_msg = Point()
                error_msg.x = 0.0
                error_msg.y = 0.0
                error_msg.z = -1.0
                self.error_pub.publish(error_msg)
                self.get_logger().info("searching for target... ")
            # 4. display the image with detected markers
            cv2.imshow("Tello Vision", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()