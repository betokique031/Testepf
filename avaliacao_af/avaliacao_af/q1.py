import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from robcomp_interfaces.msg import Conversation, TaginfoArray, YoloArray
from robcomp_util.odom import Odom
from robcomp_util.laser import Laser
from avaliacao_af.seguelinha import SegueLinha
from avaliacao_af.girar import Girar


class ExplorandoOrdenado(Node, Odom, Laser):
    """
    Exercicio 1 - AF 24b.
    Segue a linha; em cada bifurcacao le o Aruco e pergunta ao Handler
    o caminho; ao achar o animal avisa o Handler; depois volta e finaliza.
    control() eh a unica funcao que publica /cmd_vel.
    """

    def __init__(self):
        super().__init__('explorador_node')
        Odom.__init__(self)
        Laser.__init__(self)
        rclpy.spin_once(self)

        self.seguelinha_node = SegueLinha()
        self.girar_node = Girar()

        self.robot_state = 'seguir'
        self.state_machine = {
            'seguir':            self.seguir,
            'perguntar':         self.perguntar,
            'aguardar_resposta': self.aguardar_resposta,
            'reportar':          self.reportar,
            'aguardar_retorno':  self.aguardar_retorno,
            'girar_180':         self.girar_180,
            'retornar':          self.retornar,
            'stop':              self.stop,
            'done':              self.done,
        }
        self.estados_clientes = ['seguir', 'girar_180', 'retornar']
        self.twist = Twist()

        # ===== AJUSTES RAPIDOS =====
        self.ids_bifurcacao = [100, 150, 200, 250]
        self.dist_aruco = 1.2        # m: distancia pra considerar "na bifurcacao"
        self.cooldown_dist = 0.8     # m: anda isso depois de decidir antes de aceitar novo aruco
        self.dist_bias = 0.7         # m: duracao do vies de lado
        self.area_objeto = 90        # px: largura minima do box pra considerar o animal perto
        self.score_min = 0.40        # confianca minima do YOLO
        self.dist_obstaculo = 0.35   # m: para na volta se tiver placa perto na frente
        self.dist_retorno = 0.4      # m: chegou ao ponto de partida
        # ===========================

        self.mapa_animais = {'cat': 'gato', 'dog': 'cachorro', 'horse': 'cavalo'}

        self.history = []
        self.aguardando = False
        self.resposta_handler = None
        self.tags = []
        self.deteccoes = []
        self.objeto_encontrado = None
        self.ref_x = None
        self.ref_y = None
        self.x0 = self.x
        self.y0 = self.y
        self.yolo_count = 0

        self.create_subscription(Conversation, '/handler', self.handler_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(TaginfoArray, '/tag_list', self.tag_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(YoloArray, '/yolo_info', self.yolo_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.handler_pub = self.create_publisher(Conversation, '/handler', 10)
        self.poweron_yolo_pub = self.create_publisher(Bool, '/poweron_yolo', 10)

        self.timer = self.create_timer(0.1, self.control)

    # ================= CALLBACKS =================
    def handler_callback(self, msg):
        if msg.message.lower().startswith('handler:'):
            self.history = list(msg.history)
            self.resposta_handler = msg.message
            self.get_logger().info(msg.message)

    def tag_callback(self, msg):
        self.tags = msg.tags

    def yolo_callback(self, msg):
        self.deteccoes = msg.yolos

    # ================= HELPERS =================
    def liga_yolo(self):
        if self.yolo_count < 20:          # liga o YOLO algumas vezes no inicio
            m = Bool(); m.data = True
            self.poweron_yolo_pub.publish(m)
            self.yolo_count += 1

    def fala(self, texto):
        msg = Conversation()
        msg.message = f'Robo: {texto}'
        self.history.append(msg.message)
        msg.history = self.history
        self.handler_pub.publish(msg)
        self.get_logger().info(f'Robo: {texto}')

    def aruco_bifurcacao(self):
        for tag in self.tags:
            if tag.id in self.ids_bifurcacao and tag.distance < self.dist_aruco:
                return tag.id
        return None

    def animal_perto(self):
        """Animal mais confiavel (maior score) entre os que estao perto, ou None."""
        melhor, melhor_score = None, self.score_min
        for d in self.deteccoes:
            if d.classe in self.mapa_animais and (d.boxes[2] - d.boxes[0]) >= self.area_objeto:
                if d.score >= melhor_score:
                    melhor_score = d.score
                    melhor = self.mapa_animais[d.classe]
        return melhor

    def dist_de(self, x, y):
        return np.hypot(self.x - x, self.y - y)

    # ================= ESTADOS =================
    def seguir(self):
        self.liga_yolo()
        if self.seguelinha_node.robot_state == 'done':
            rclpy.spin_once(self.seguelinha_node)
            self.seguelinha_node.reset()
        rclpy.spin_once(self.seguelinha_node)

        # tira o vies de lado depois de andar dist_bias
        if self.seguelinha_node.lado and self.ref_x is not None:
            if self.dist_de(self.ref_x, self.ref_y) > self.dist_bias:
                self.seguelinha_node.lado = None

        # achou o animal?
        nome = self.animal_perto()
        if nome:
            self.objeto_encontrado = nome
            self.seguelinha_node.robot_state = 'stop'
            rclpy.spin_once(self.seguelinha_node)
            self.robot_state = 'reportar'
            return

        # chegou na bifurcacao?
        passou_cooldown = self.ref_x is None or self.dist_de(self.ref_x, self.ref_y) > self.cooldown_dist
        bif = self.aruco_bifurcacao()
        if bif and passou_cooldown:
            self.id_bifurcacao = bif
            self.seguelinha_node.robot_state = 'stop'
            rclpy.spin_once(self.seguelinha_node)
            self.robot_state = 'perguntar'

    def perguntar(self):
        if not self.aguardando:
            self.resposta_handler = None
            self.fala(f'Bifurcação: {self.id_bifurcacao}')
            self.aguardando = True
        self.robot_state = 'aguardar_resposta'

    def aguardar_resposta(self):
        self.twist = Twist()
        if self.resposta_handler is None:
            return
        msg = self.resposta_handler.lower()
        if 'direita' in msg:
            self.seguelinha_node.lado = 'direita'
        elif 'esquerda' in msg:
            self.seguelinha_node.lado = 'esquerda'
        else:
            return
        self.ref_x, self.ref_y = self.x, self.y
        self.aguardando = False
        self.resposta_handler = None
        self.robot_state = 'seguir'

    def reportar(self):
        if not self.aguardando:
            self.resposta_handler = None
            self.fala(f'Objeto: {self.objeto_encontrado}')
            self.aguardando = True
        self.robot_state = 'aguardar_retorno'

    def aguardar_retorno(self):
        self.twist = Twist()
        if self.resposta_handler and 'retorne' in self.resposta_handler.lower():
            self.aguardando = False
            self.resposta_handler = None
            self.robot_state = 'girar_180'

    def girar_180(self):
        if self.girar_node.robot_state == 'done':
            rclpy.spin_once(self.girar_node)
            self.girar_node.reset(rotacao=np.pi)
        rclpy.spin_once(self.girar_node)
        if self.girar_node.robot_state == 'done':
            self.girar_node.control()
            self.seguelinha_node.lado = None
            self.ref_x = self.ref_y = None
            self.robot_state = 'retornar'

    def retornar(self):
        if self.seguelinha_node.robot_state == 'done':
            rclpy.spin_once(self.seguelinha_node)
            self.seguelinha_node.reset()
        rclpy.spin_once(self.seguelinha_node)

        # chegou no inicio OU tem placa perto na frente -> para sem bater
        perto_inicio = self.dist_de(self.x0, self.y0) < self.dist_retorno
        placa_na_frente = self.front and min(self.front) < self.dist_obstaculo
        if perto_inicio or placa_na_frente:
            self.seguelinha_node.robot_state = 'stop'
            rclpy.spin_once(self.seguelinha_node)
            self.robot_state = 'stop'

    def stop(self):
        self.twist = Twist()
        self.get_logger().info('Fim.')
        self.robot_state = 'done'

    def done(self):
        self.twist = Twist()

    # ================= CONTROL (IDENTICO ao base_control) =================
    def control(self):
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()
        if self.robot_state not in self.estados_clientes:
            self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    ros_node = ExplorandoOrdenado()
    while rclpy.ok():
        rclpy.spin_once(ros_node)
        if ros_node.robot_state == 'done':
            ros_node.cmd_vel_pub.publish(Twist())
    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
