import rclpy
import cv2 as cv
from cv_bridge import CvBridge
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from tello_msgs.srv import TelloAction

class VisualServoing(Node):

    def __init__(self):
        super().__init__('aruco_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.subscriber = self.create_subscription(Image,'/image_raw',self.listener_callback,10)
        self.client = self.create_client(TelloAction, 'tello_action')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TelloAction.Request()
        
        self.send_request('takeoff')
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
        self.parameters = cv.aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(self.dictionary, self.parameters)
        self.bridge = CvBridge()
        
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        markerCorners, markerIds, rejectedCandidates = self.detector.detectMarkers(cv_image)
        
        if markerIds is not None:
            print("Aruco marker detected!!!!!!!!!")
            future = self.send_request('land')
            if future.result() is not None:
                self.get_logger().info('Result of tello_action: %s' % (future.result()))
            else:
                self.get_logger().error('Exception while calling service: %r' % future.exception())
            image_with_markers = cv.aruco.drawDetectedMarkers(cv_image.copy(), markerCorners, markerIds)
        else:
            print("No Aruco marker detected.")
            
        #cv.imshow('image', cv_image)
        #cv.waitKey(1)
        
    def send_request(self, command):
        self.req.cmd = command
        self.future = self.client.call_async(self.req)
        #rclpy.spin_until_future_complete(self, self.future)
        return self.future
     
def main(args=None):
    rclpy.init(args=args)

    aruco_node = VisualServoing()

    rclpy.spin(aruco_node)

    aruco_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
