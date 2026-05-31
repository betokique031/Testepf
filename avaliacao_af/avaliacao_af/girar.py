import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robcomp_util.odom import Odom


class Girar(Node, Odom):
    """Nó de AÇÃO: gira `rotacao` radianos usando odometria (controle P no yaw)."""

    def __init__(self):
        super().__init__('girar_node')
        Odom.__init__(self)
        self.timer = None

        self.robot_state = 'done'
        self.state_machine = {
            'girar': self.girar,
            'stop': self.stop,
            'done': self.done,
        }

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def ajuste_angulo(self, angulo):
        return np.arctan2(np.sin(angulo), np.cos(angulo))

    def reset(self, rotacao):
        # rotacao em RADIANOS (ex.: np.pi para 180 graus)
        self.twist = Twist()
        self.robot_state = 'girar'
        if self.timer is None:
            self.timer = self.create_timer(0.1, self.control)
        self.goal_yaw = self.ajuste_angulo(self.yaw + rotacao)

    def girar(self):
        erro = self.ajuste_angulo(self.goal_yaw - self.yaw)
        if abs(erro) < np.deg2rad(5):
            self.robot_state = 'stop'
            return
        self.twist.angular.z = 0.5 * erro
        self.twist.linear.x = 0.0

    def stop(self):
        self.twist = Twist()
        self.timer.cancel()
        self.timer = None
        self.robot_state = 'done'

    def done(self):
        self.twist = Twist()

    def control(self):
        self.state_machine[self.robot_state]()
        self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    ros_node = Girar()
    rclpy.spin_once(ros_node)
    ros_node.reset(rotacao=np.pi)
    while not ros_node.robot_state == 'done':
        rclpy.spin_once(ros_node, timeout_sec=1)
    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
