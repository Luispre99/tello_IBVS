import rclpy
from pynput import keyboard
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from tello_msgs.srv import TelloAction
import yaml
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge

def Proyection_2D(camera_matrix, top_left, top_right, bottom_right, bottom_left):
    fxy = np.array([camera_matrix[0, 0], camera_matrix[1, 1]])
    cxy = np.array([camera_matrix[0, 2], camera_matrix[1, 2]])
    s1 = (top_left - cxy) / fxy
    s2 = (top_right - cxy) / fxy
    s3 = (bottom_right - cxy) / fxy
    s4 = (bottom_left - cxy) / fxy
    s_con = np.concatenate([s1, s2, s3, s4]).astype(float)
    s = s_con.reshape(-1, 1)
    return s

def Interaction_Matrix(s, Z):
    x = s[0].item()
    y = s[1].item()
    L = np.array([[-1 / Z, 0, x / Z, x * y, -(1 + x ** 2), y],
                  [0, -1 / Z, y / Z, 1 + y ** 2, -x * y, -x]])
    return L

class ArucoNode(Node):

    # ROS Node Constructor
    def __init__(self):
        super().__init__('aruco_node')

        # Crear Subscriptor, Publicador y Cliente
        self.subscriber = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.client = self.create_client(TelloAction, 'tello_action')
        self.pressed_keys = set()

        # Revisar si los servicios del dron estan disponibles
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TelloAction.Request()

        # Enviar petición de despegue
        self.send_request('takeoff')  # ⚠️ CAUTION: Uncommenting WILL trigger drone takeoff.

        # Parámetros para detección de ArUco
        self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_250)
        self.parameters = cv.aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(self.dictionary, self.parameters)
        self.bridge = CvBridge()

        # Abrir parámetros intrínsecos del dron en archivo .yml
        with open('/home/modular/tello_ros_ws/src/tello_ibvs/tello_ibvs/tello_mirror_2.yaml') as file:
            camera_params = yaml.safe_load(file)

        # Extraer la matriz de la cámara y los coeficientes de distorsión
        self.camera_matrix = np.array(camera_params.get('camera_matrix').get('data')).reshape(3, 3)
        self.dist_coeffs = np.array(camera_params.get('distortion_coefficients').get('data'))

        # Definir tamaño del ArUco
        # self.aruco_size = 0.096 #Aruco Mediano 5x5
        self.aruco_size = 0.180 #Aruco Grande 5x5

        # Definir coordenadas del ArUco dependiendo del tamaño
        self.markerPoints = self.aruco_size * np.array([[0, 0, 0], [0, 1, 0], [1, 1, 0], [1, 0, 0]], dtype=np.float32)
        
        self.P = np.array([[0,               0, self.aruco_size, self.aruco_size],
                           [0, self.aruco_size, self.aruco_size,               0],
                           [0,               0,               0,               0],
                           [1,               1,               1,               1]])
        self.R = np.array([[0, -1, 0],
                           [0, 0, -1],
                           [1, 0, 0]]).astype(float) # Matriz de rotacion sin el espejo
        # self.R = np.array([[0, -1, 0],
        #                    [-1, 0, 0],
        #                    [0, 0, -1]]).astype(float) # Matriz de rotacion con el espejo

        # Definir distancia del centro focal de la camara al centro de rotación del dron
        self.t = np.array([0, 0, -0.036]).astype(float)  # Sin espejo
        # self.t = np.array([0,0.0425, 0]).astype(float)   # Con espejo

        # Definir matriz antisimétrica asociada al vector de traslación t
        self.St = np.array([[         0, -self.t[2],  self.t[1]],
                            [ self.t[2],          0, -self.t[0]],
                            [-self.t[1],  self.t[0],         0]]).astype(float)
        
        # Definir la matriz de transformación de movimiento espacial (V)
        self.StR = np.dot(self.St, self.R).astype(float)
        self.m_zeros = np.zeros((3, 3))
        self.V = np.block([[self.R, self.StR], [self.m_zeros, self.R]]).astype(float)

        self.vc = np.zeros((6, 1))

        print("Press Enter when the Marker is in Place")
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        # This flag allows to change between peviously defined setpoint (sd) or define it by taking off the drone and press enter on a desired ArUco position
        #   True  : The set point (sd) will be defined when you hit enter after the drone take off based on the Aruco position
        #   False : The set point (sd) will be defined before flight based on a square in the image
        self.dynamic_sd_flag = True
        
        # Fixed set point definition
        w = 960
        h = 720
        c = 80
        self.points = np.array([[round(w/2-c), round(h/2-c)], [round(w/2+c), round(h/2-c)], [round(w/2+c), round(h/2+c)], [round(w/2-c), round(h/2+c)]])

        # Get the 2D Proyection for the fixed desired points
        self.sd = Proyection_2D(self.camera_matrix, self.points[0], self.points[1], self.points[2], self.points[3])
        self.sd = np.zeros((8, 1))
        self.sd = self.sd.reshape(-1, 1)
        
        # Flag to know when the user pressed Enter key
        self.enter_flag = False

    # Callback for Tello Image
    def listener_callback(self, msg):

        cv_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # cv_frame = cv.flip(cv_frame, 0) # Cambia el sentido de la imagen, para cuando utilizamos el espejo

        corners, ids, rejected = self.detector.detectMarkers(cv_frame)

        # If dynamic setpoint is expected
        if self.dynamic_sd_flag is True:
            
            # Check if ArUco marker was detected
            if len(corners) > 0:
                
                # Check if Enter key was pushed
                if self.enter_flag is True:
                    top_left = corners[0][0][0]
                    top_right = corners[0][0][1]
                    bottom_right = corners[0][0][2]
                    bottom_left = corners[0][0][3]
                    self.sd = Proyection_2D(self.camera_matrix, top_left, top_right, bottom_right, bottom_left)
                    self.points = corners[0][0]
                    self.dynamic_sd_flag = True

        # If previously defined setpoint is expected or dynamic setpoint was already defined
        else:

            # Marcar los puntos deseados en la imagen
            for point in self.points:
                point = tuple(point.astype(int))
                cv_frame = cv.circle(cv_frame, point, radius=5, color=(0, 0, 255), thickness=-1)

            # Si se detectó un ArUco, efectuar el algoritmo de visual servoing
            if len(corners) > 0:
                for i in range(0, len(ids)):

                    # Extraer vector de rotación (rvec) y traslación (tvec) del marco de referencia de la cámara al ArUco
                    retval, rvec, tvec = cv.solvePnP(self.markerPoints, corners[i], self.camera_matrix,
                                                     self.dist_coeffs)
                    cv_frame = cv.drawFrameAxes(cv_frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

                    T = np.eye(4)
                    R, jac = cv.Rodrigues(rvec) # Convertir vector de Rodrigues a matriz de rotación
                    T[:3, :3] = R
                    T[:3, 3] = tvec.squeeze()   # Añadir vector de traslación a la matriz 
                    Z = np.dot(T[2, :], self.P) # Calcular la profundidad del ArUco a la cámara
                    # print(Z)

                    # Esquinas de la imagen
                    top_left = corners[0][0][0]
                    top_right = corners[0][0][1]
                    bottom_right = corners[0][0][2]
                    bottom_left = corners[0][0][3]

                    # Proyección 2D de las esquinas del ArUco
                    s = Proyection_2D(self.camera_matrix, top_left, top_right, bottom_right, bottom_left)
                    # print("valor de s", s)

                    # Matriz de Interacción
                    L1 = Interaction_Matrix(s[0:2], Z[0])
                    L2 = Interaction_Matrix(s[2:4], Z[1])
                    L3 = Interaction_Matrix(s[4:6], Z[2])
                    L4 = Interaction_Matrix(s[6:8], Z[3])
                    L = np.concatenate([L1, L2, L3, L4]).astype(float)
                    # print("matriz L:",L.shape)

                    # Mapeo de velocidades Camara-Dron
                    LV = np.dot(L, self.V)

                    # IBVS
                    e = s - self.sd            # Error
                    lam = 0.8  # sin espejo    # Lambda
                    # lam = 1    # con espejo
                    invLV = np.linalg.pinv(LV) # Pseudo-inversa
                    inveLV = np.dot(invLV, e)

                    self.vc = -lam * inveLV    # Calculo de velocidades del dron
                    print("v_c: ", self.vc)
                    self.publish_velocity() #Publicador que manda las velocidades al dron

            # Condicion para que se quede en ceros cuando no detecta el aruco la camara de tal forma que este no se mueva
            else:
                self.vc = np.zeros((6, 1))
                self.publish_velocity()

        # Mostrar frame del dron
        cv.imshow('image', cv_frame)
        cv.waitKey(1)

    # Send request to Tello Service
    def send_request(self, command):
        self.req.cmd = command
        self.future = self.client.call_async(self.req)
        # rclpy.spin_until_future_complete(self, self.future)
        return self.future

    # Publish Tello velocity (cmd_vel)
    def publish_velocity(self):
        twist_msg = Twist()
        twist_msg.linear.x = float(self.vc[0])
        twist_msg.linear.y = float(self.vc[1])
        twist_msg.linear.z = float(self.vc[2])
        twist_msg.angular.z = float(self.vc[5])
        self.publisher.publish(twist_msg)

    # Raise Flag when Enter is pressed 
    def on_press(self, key):
        if key == keyboard.Key.enter:
            self.enter_flag = not self.enter_flag
            print(key)

def main(args=None):
    rclpy.init(args=args)

    aruco_node = ArucoNode()

    rclpy.spin(aruco_node)

    aruco_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
