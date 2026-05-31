import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge


class SegueLinha(Node):
    """
    No de ACAO: segue a linha do mapa divisoes.

    A linha e colorida (azul/vermelho) sobre chao de madeira. Como o chao
    tem cor "fraca" e a linha tem cor "forte", basta pegar qualquer pixel
    com saturacao e brilho altos -> isso isola a linha.

    self.lado : None / 'direita' / 'esquerda'  -> vies na bifurcacao
    self.cx, self.erro, self.tem_linha
    """

    def __init__(self):
        super().__init__('seguelinha_node')
        self.timer = None

        self.robot_state = 'done'
        self.state_machine = {
            'seguir': self.seguir,
            'stop': self.stop,
            'done': self.done,
        }

        # ===== AJUSTES RAPIDOS (mexa aqui na prova se precisar) =====
        self.s_min = 113      # saturacao minima da linha
        self.v_min = 138      # brilho minimo da linha
        self.kp = 1.0         # ganho do giro (menor = menos zigue-zague)
        self.v_linear = 0.08  # velocidade pra frente (menor = mais estavel)
        self.w_max = 0.35     # giro maximo
        # ============================================================

        self.bridge = CvBridge()
        self.twist = Twist()
        self.lado = None
        self.cx = None
        self.w = None
        self.erro = None
        self.tem_linha = False

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def image_callback(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        h, width = img.shape[:2]
        self.w = width / 2

        roi = img[h // 2:, :]                       # metade de baixo
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower = np.array([0, self.s_min, self.v_min])
        upper = np.array([179, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)       # qualquer cor forte = linha

        # vies de lado na bifurcacao
        meio = mask.shape[1] // 2
        if self.lado == 'direita':
            mask[:, :meio] = 0
        elif self.lado == 'esquerda':
            mask[:, meio:] = 0

        M = cv2.moments(mask)
        if M['m00'] > 0:
            self.cx = int(M['m10'] / M['m00'])
            self.erro = -(self.cx - self.w) / self.w
            self.tem_linha = True
            cv2.circle(roi, (self.cx, roi.shape[0] // 2), 8, (0, 255, 0), -1)
        else:
            self.cx = None
            self.erro = None
            self.tem_linha = False

        cv2.imshow('SegueLinha', img)
        cv2.imshow('Mask', mask)
        cv2.waitKey(1)

    def reset(self):
        self.twist = Twist()
        self.robot_state = 'seguir'
        if self.timer is None:
            self.timer = self.create_timer(0.1, self.control)

    def seguir(self):
        if self.erro is None:               # perdeu a linha -> anda devagar reto
            self.twist.linear.x = 0.05
            self.twist.angular.z = 0.0
            return
        w = self.kp * self.erro
        w = max(-self.w_max, min(self.w_max, w))
        self.twist.linear.x = self.v_linear
        self.twist.angular.z = float(w)

    def stop(self):
        self.twist = Twist()
        self.timer.cancel()
        self.timer = None
        self.robot_state = 'done'

    def done(self):
        self.twist = Twist()

    def control(self):
        self.twist = Twist()
        self.state_machine[self.robot_state]()
        self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    ros_node = SegueLinha()
    rclpy.spin_once(ros_node, timeout_sec=1)
    ros_node.reset()
    while not ros_node.robot_state == 'done':
        rclpy.spin_once(ros_node, timeout_sec=1)
    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
