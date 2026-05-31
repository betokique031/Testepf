import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robcomp_util.odom import Odom
from robcomp_util.laser import Laser


class MedirCaixa(Node, Odom, Laser):
    """
    Exercicio 2 - AF 24b.
    Navega ate a caixa, contorna mantendo-a a ESQUERDA usando o laser,
    mede largura e comprimento (odometria entre os cantos) e volta ao
    ponto de partida. A funcao control() eh a unica a publicar /cmd_vel.

    Ideia da medida: contornando o retangulo a uma distancia fixa `d`,
    os trechos RETOS da trajetoria tem o mesmo comprimento das faces da
    caixa (os cantos viram arcos). Mede-se o trecho reto entre dois cantos
    via odometria. Cantos = quando o laser da esquerda "abre" (some a face).
    """

    def __init__(self):
        super().__init__('medidor_node')
        Odom.__init__(self)
        Laser.__init__(self)
        rclpy.spin_once(self)

        self.robot_state = 'procurar'
        self.state_machine = {
            'procurar':  self.procurar,
            'aproximar': self.aproximar,
            'encostar':  self.encostar,
            'contornar': self.contornar,
            'retornar':  self.retornar,
            'stop':      self.stop,
            'done':      self.done,
        }

        # ---- Parametros (AJUSTE NA PROVA) ----
        self.d = 0.45          # m: distancia que mantem da caixa
        self.v = 0.15          # m/s: velocidade ao contornar
        self.kp_ang = 1.5      # ganho do controle lateral (laser esquerda)
        self.abre_corner = 0.6 # m: variacao na esquerda que indica canto (face acabou)
        self.tol_front = 0.1   # m: folga para detectar parede a frente

        # ---- Estado interno ----
        self.twist = Twist()
        self.x0 = self.x       # ponto de partida
        self.y0 = self.y
        self.cantos = 0
        self.seg_inicio = None             # (x,y) do inicio do trecho reto atual
        self.medidas = []                  # comprimentos dos trechos retos
        self.esq_anterior = None

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control)

    # ================= HELPERS =================
    def front_min(self):
        vals = [x for x in self.front if np.isfinite(x)]
        return min(vals) if vals else np.inf

    def esq_min(self):
        vals = [x for x in self.left if np.isfinite(x)]
        return min(vals) if vals else np.inf

    def idx_mais_perto(self):
        """Indice (graus) da menor leitura do laser -> direcao da caixa."""
        arr = np.array([x if np.isfinite(x) else 1e6 for x in self.laser_msg])
        return int(np.argmin(arr))

    def dist_de(self, x, y):
        return np.hypot(self.x - x, self.y - y)

    # ================= ESTADOS =================
    def procurar(self):
        """Gira ate ficar de frente para a caixa (menor leitura ~ 0 graus)."""
        rclpy.spin_once(self, timeout_sec=0.1)
        idx = self.idx_mais_perto()
        ang = idx if idx <= 180 else idx - 360   # graus, [-180,180]
        if abs(ang) < 5:
            self.twist = Twist()
            self.robot_state = 'aproximar'
        else:
            self.twist.angular.z = 0.3 * np.sign(ang)

    def aproximar(self):
        """Anda em direcao a caixa ate ficar a `d` dela."""
        rclpy.spin_once(self, timeout_sec=0.1)
        idx = self.idx_mais_perto()
        ang = idx if idx <= 180 else idx - 360
        if self.front_min() <= self.d:
            self.twist = Twist()
            self.robot_state = 'encostar'
        else:
            self.twist.linear.x = self.v
            self.twist.angular.z = 0.01 * ang   # corrige para apontar pra caixa

    def encostar(self):
        """Gira 90 graus para a direita -> caixa fica na esquerda do robo."""
        rclpy.spin_once(self, timeout_sec=0.1)
        if not hasattr(self, 'goal_yaw_enc'):
            self.goal_yaw_enc = np.arctan2(
                np.sin(self.yaw - np.pi / 2), np.cos(self.yaw - np.pi / 2))
        erro = np.arctan2(np.sin(self.goal_yaw_enc - self.yaw),
                          np.cos(self.goal_yaw_enc - self.yaw))
        if abs(erro) < np.deg2rad(5):
            self.twist = Twist()
            self.seg_inicio = (self.x, self.y)
            self.esq_anterior = self.esq_min()
            self.robot_state = 'contornar'
        else:
            self.twist.angular.z = 0.5 * erro

    def contornar(self):
        """Segue a caixa pela esquerda. Conta cantos e mede os trechos retos."""
        rclpy.spin_once(self, timeout_sec=0.1)
        esq = self.esq_min()
        frente = self.front_min()

        # parede a frente (canto interno) -> gira para a direita, sem andar
        if frente < self.d + self.tol_front:
            self.twist.linear.x = 0.0
            self.twist.angular.z = -0.4
            return

        # face acabou (a esquerda "abriu") -> e um canto da caixa
        if self.esq_anterior is not None and (esq - self.esq_anterior) > self.abre_corner:
            comp = self.dist_de(*self.seg_inicio)
            self.medidas.append(comp)
            self.cantos += 1
            self.get_logger().info(
                f'Canto {self.cantos}: trecho medido = {comp:.2f} m')
            self.seg_inicio = (self.x, self.y)
            # contornou 4 faces -> terminou
            if self.cantos >= 4:
                self.reporta_medidas()
                self.robot_state = 'retornar'
                return
            # arco para a esquerda para envolver o canto
            self.twist.linear.x = self.v
            self.twist.angular.z = 0.6
            self.esq_anterior = esq
            return

        # segue reto controlando a distancia lateral (mantem caixa a `d`)
        erro = esq - self.d
        self.twist.linear.x = self.v
        self.twist.angular.z = float(np.clip(self.kp_ang * erro, -0.5, 0.5))
        self.esq_anterior = esq

    def reporta_medidas(self):
        if len(self.medidas) >= 4:
            largura = (self.medidas[0] + self.medidas[2]) / 2
            comprimento = (self.medidas[1] + self.medidas[3]) / 2
        else:
            largura = self.medidas[0] if self.medidas else 0.0
            comprimento = self.medidas[1] if len(self.medidas) > 1 else 0.0
        self.get_logger().info(
            f'>>> CAIXA: largura ~ {largura:.2f} m | comprimento ~ {comprimento:.2f} m')

    def retornar(self):
        """Volta ao ponto de partida (go-to-point por odometria)."""
        rclpy.spin_once(self, timeout_sec=0.1)
        if self.dist_de(self.x0, self.y0) < 0.2:
            self.twist = Twist()
            self.robot_state = 'stop'
            return
        ang_alvo = np.arctan2(self.y0 - self.y, self.x0 - self.x)
        erro = np.arctan2(np.sin(ang_alvo - self.yaw), np.cos(ang_alvo - self.yaw))
        if abs(erro) > np.deg2rad(15):
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.5 * erro
        else:
            self.twist.linear.x = self.v
            self.twist.angular.z = 0.5 * erro

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
