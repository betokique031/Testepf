import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from robcomp_util.odom import Odom


class MedirCaixa(Node, Odom):
    """
    Exercicio 2 - AF 24b.
    Acha a caixa pelo laser, ORBITA ela mantendo-a a esquerda a uma
    distancia fixa e acumula os pontos do laser. A medida e a extensao
    dos pontos (max - min), entao nao depende de onde a odometria comecou:
        largura     = max(x) - min(x)
        comprimento = max(y) - min(y)
    Depois volta ao ponto de partida. control() e a unica que publica /cmd_vel.
    """

    def __init__(self):
        super().__init__('medidor_node')
        Odom.__init__(self)
        rclpy.spin_once(self)

        self.robot_state = 'aproximar'
        self.state_machine = {
            'aproximar': self.aproximar,
            'orbitar':   self.orbitar,
            'retornar':  self.retornar,
            'stop':      self.stop,
            'done':      self.done,
        }

        # ===== AJUSTES RAPIDOS =====
        self.D = 0.7          # m: distancia que mantem da caixa
        self.v = 0.12         # m/s: velocidade
        self.kb = 0.8         # ganho pra manter a caixa a 90 graus (esquerda)
        self.kd = 1.5         # ganho pra manter a distancia D
        # ===========================

        self.twist = Twist()
        self.x0, self.y0 = self.x, self.y

        self.ranges = None
        self.angle_min = 0.0
        self.angle_inc = 0.0
        self.range_max = 3.5

        self.xs = []
        self.ys = []

        self.ox = self.oy = None
        self.afastou = False

        self.create_subscription(
            LaserScan, '/scan', self.scan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control)

    # ================= CALLBACK =================
    def scan_callback(self, msg):
        self.ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_inc = msg.angle_increment
        self.range_max = msg.range_max

    # ================= HELPERS =================
    def ajuste(self, a):
        return np.arctan2(np.sin(a), np.cos(a))

    def pronto(self):
        return self.ranges is not None

    def feixes(self):
        lim = 3.4   # alcance util do LDS (~3.5 m); nao depende do range_max reportado
        for i, r in enumerate(self.ranges):
            if np.isfinite(r) and 0.05 < r < lim:
                yield self.ajuste(self.angle_min + i * self.angle_inc), r

    def ponto_mais_proximo(self):
        """(angulo_no_robo, distancia) do ponto mais perto = caixa. (None, inf) se nao ve."""
        melhor_a, melhor_r = None, np.inf
        for ang, r in self.feixes():
            if r < melhor_r:
                melhor_r, melhor_a = r, ang
        return melhor_a, melhor_r

    def coleta(self):
        for ang, r in self.feixes():
            a = self.yaw + ang
            self.xs.append(self.x + r * np.cos(a))
            self.ys.append(self.y + r * np.sin(a))

    def vai_para(self, tx, ty):
        dx, dy = tx - self.x, ty - self.y
        if np.hypot(dx, dy) < 0.2:
            return True
        erro = self.ajuste(np.arctan2(dy, dx) - self.yaw)
        self.twist.linear.x = 0.0 if abs(erro) > 0.4 else self.v
        self.twist.angular.z = float(np.clip(1.0 * erro, -0.8, 0.8))
        return False

    def reporta(self):
        if len(self.xs) < 10:
            self.get_logger().warn('poucos pontos para medir a caixa')
            return
        xs = np.array(self.xs)
        ys = np.array(self.ys)
        # percentis 1 e 99 descartam pontos espurios (1 feixe perdido nao estraga a medida)
        x_lo, x_hi = np.percentile(xs, 1), np.percentile(xs, 99)
        y_lo, y_hi = np.percentile(ys, 1), np.percentile(ys, 99)
        largura = x_hi - x_lo
        comprimento = y_hi - y_lo
        self.get_logger().info(
            f'>>> CAIXA: largura (X) ~ {largura:.2f} m | comprimento (Y) ~ {comprimento:.2f} m')
        self.get_logger().info(
            f'    [debug] pontos={len(xs)} x=[{x_lo:.2f},{x_hi:.2f}] y=[{y_lo:.2f},{y_hi:.2f}]')

    # ================= ESTADOS =================
    def aproximar(self):
        """Anda em direcao a caixa (sempre pra frente) ate ficar a D dela."""
        if not self.pronto():
            return
        ang, dist = self.ponto_mais_proximo()
        if ang is None:                          # nao ve a caixa -> diagnostica e procura
            finitos = [r for r in self.ranges if np.isfinite(r) and 0.05 < r < 3.4]
            if finitos:
                self.get_logger().warn(
                    f'caixa fora do filtro? leituras_uteis={len(finitos)} min={min(finitos):.2f}')
            else:
                self.get_logger().warn(
                    'NENHUMA leitura util do laser (caixa nao spawnou, robo em cima dela, ou fora de alcance)')
            self.twist.angular.z = 0.4
            return
        if dist <= self.D:                       # chegou perto -> comeca a orbitar
            self.ox, self.oy = self.x, self.y
            self.afastou = False
            self.robot_state = 'orbitar'
            return
        self.twist.linear.x = self.v             # SEMPRE anda pra frente
        self.twist.angular.z = float(np.clip(1.0 * ang, -0.6, 0.6))

    def orbitar(self):
        """Orbita mantendo a caixa a ~90 graus (esquerda) e a distancia D."""
        if not self.pronto():
            return
        self.coleta()
        beta, rho = self.ponto_mais_proximo()

        if beta is None:                         # perdeu a caixa -> curva pra esquerda procurando
            self.twist.linear.x = self.v
            self.twist.angular.z = 0.4
        else:
            ang = self.kb * (beta - np.pi / 2) + self.kd * (rho - self.D)
            self.twist.linear.x = self.v
            self.twist.angular.z = float(np.clip(ang, -0.8, 0.8))

        # terminou a volta? (afastou do inicio e voltou)
        d = np.hypot(self.x - self.ox, self.y - self.oy)
        if d > 0.8:
            self.afastou = True
        if self.afastou and d < 0.3:
            self.reporta()
            self.robot_state = 'retornar'

    def retornar(self):
        if self.vai_para(self.x0, self.y0):
            self.robot_state = 'stop'

    def stop(self):
        self.twist = Twist()
        self.robot_state = 'done'

    def done(self):
        self.twist = Twist()

    # ================= CONTROL (unica a publicar cmd_vel) =================
    def control(self):
        self.twist = Twist()
        self.state_machine[self.robot_state]()
        self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    ros_node = MedirCaixa()
    while rclpy.ok():
        rclpy.spin_once(ros_node)
        if ros_node.robot_state == 'done':
            ros_node.cmd_vel_pub.publish(Twist())
    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
