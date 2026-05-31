import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

"""
Tuner da mascara da linha (azul + vermelho) do mapa divisoes.
NAO move o robo. Mexa nos sliders ate a linha aparecer branca e limpa na
janela 'Mask'. Depois copie os valores S_min / V_min impressos no terminal
para o seguelinha.py.

    ros2 run avaliacao_af tuner
"""


class Tuner(Node):
    def __init__(self):
        super().__init__('tuner_node')
        self.bridge = CvBridge()
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        cv2.namedWindow('Mask')
        cv2.createTrackbar('S_min', 'Mask', 80, 255, lambda v: None)
        cv2.createTrackbar('V_min', 'Mask', 50, 255, lambda v: None)
        # se quiser, mexa tambem nas bordas de matiz:
        cv2.createTrackbar('Blue_lo', 'Mask', 95, 179, lambda v: None)
        cv2.createTrackbar('Blue_hi', 'Mask', 135, 179, lambda v: None)

        self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def cb(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        h = img.shape[0]
        roi = img[h // 2:, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        s = cv2.getTrackbarPos('S_min', 'Mask')
        v = cv2.getTrackbarPos('V_min', 'Mask')
        blo = cv2.getTrackbarPos('Blue_lo', 'Mask')
        bhi = cv2.getTrackbarPos('Blue_hi', 'Mask')

        red1 = cv2.inRange(hsv, np.array([0,   s, v]), np.array([10,  255, 255]))
        red2 = cv2.inRange(hsv, np.array([170, s, v]), np.array([180, 255, 255]))
        blue = cv2.inRange(hsv, np.array([blo, s, v]), np.array([bhi, 255, 255]))
        mask = red1 | red2 | blue
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        area = int(cv2.moments(mask)['m00'] / 255)
        print(f'S_min={s} V_min={v} Blue=[{blo},{bhi}] | area_linha={area}')

        cv2.imshow('ROI', roi)
        cv2.imshow('Mask', mask)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = Tuner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
