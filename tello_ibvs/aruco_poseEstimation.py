import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from tello_msgs.srv import TelloAction

import yaml
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge

class ArucoNode(Node):

    def __init__(self):
        super().__init__('aruco_node')
        self.subscriber = self.create_subscription(Image,'/image_raw',self.listener_callback,10)
        self.client = self.create_client(TelloAction, 'tello_action')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TelloAction.Request()
        
        #self.send_request('takeoff') # ⚠️ CAUTION: Uncommenting WILL trigger drone takeoff.
        
        self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_100)
        self.parameters = cv.aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(self.dictionary, self.parameters)
        self.bridge = CvBridge()

        with open('/home/modular/tello_ros_ws/src/tello_ibvs/tello_ibvs/test.yaml') as file:
            camera_params = yaml.load(file)
        self.camera_matrix = np.array(camera_params.get('camera_matrix').get('data')).reshape(3,3)
        self.dist_coeffs = np.array(camera_params.get('distortion_coefficients').get('data'))

        self.aruco_size = 0.095
        self.markerPoints = self.aruco_size*np.array([[0, 0, 0], [0, 1, 0], [1, 1, 0], [1, 0, 0]], dtype=np.float32)

    def listener_callback(self, msg):
        cv_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        gray = cv.cvtColor(cv_frame, cv.COLOR_BGR2GRAY)

        markerCorners, markerIds, rejectedCandidates = self.detector.detectMarkers(cv_frame)

        if len(markerCorners) > 0:
            for i in range(0, len(markerIds)):
                retval, rvec, tvec = cv.solvePnP(self.markerPoints, markerCorners[i], self.camera_matrix, self.dist_coeffs)
                cv_frame = cv.drawFrameAxes(cv_frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                print(tvec)

        cv.imshow('image', cv_frame)
        cv.waitKey(1)
        
    def send_request(self, command):
        self.req.cmd = command
        self.future = self.client.call_async(self.req)
        #rclpy.spin_until_future_complete(self, self.future)
        return self.future
     
def main(args=None):
    rclpy.init(args=args)

    aruco_node = ArucoNode()

    rclpy.spin(aruco_node)

    aruco_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
