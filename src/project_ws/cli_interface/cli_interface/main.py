import typer
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import inquirer
import threading
import sys, os
import time

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

app = typer.Typer()

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            qos_profile=qos_profile_sensor_data
        )

        self.connected = False
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.killed = False
        self.safety_distance = 0.35  # Reduzida a distância de parada pela metade
        self.front_clear = True
        self.back_clear = True

    def lidar_callback(self, msg):
        num_ranges = len(msg.ranges)
        sector_size = num_ranges // 12  # Ajustado para tornar o cone mais estreito

        # Definir índices dos setores usando a lógica de divisão
        front_left_indices = range(num_ranges - sector_size, num_ranges)  # Parte frontal esquerda
        front_right_indices = range(0, sector_size)  # Parte frontal direita
        back_indices = range(5 * sector_size, 7 * sector_size)  # Parte traseira

        front_ranges = []
        back_ranges = []

        # Coletar distâncias frontais esquerdas
        for index in front_left_indices:
            if 0.01 < msg.ranges[index] < 100.0:  # Garantir que a distância esteja dentro dos limites válidos
                front_ranges.append(msg.ranges[index])

        # Coletar distâncias frontais direitas
        for index in front_right_indices:
            if 0.01 < msg.ranges[index] < 100.0:  # Garantir que a distância esteja dentro dos limites válidos
                front_ranges.append(msg.ranges[index])

        # Coletar distâncias traseiras
        for index in back_indices:
            if 0.01 < msg.ranges[index] < 100.0:  # Garantir que a distância esteja dentro dos limites válidos
                back_ranges.append(msg.ranges[index])

        # Verificar se alguma das distâncias nos ranges frontais ou traseiros está abaixo do limite de segurança
        if any(r < self.safety_distance for r in front_ranges):
            self.front_clear = False
        else:
            self.front_clear = True

        if any(r < self.safety_distance for r in back_ranges):
            self.back_clear = False
        else:
            self.back_clear = True

        if not self.front_clear and self.linear_speed > 0:
            self.stop_robot()
            print(f"Obstáculo detectado perto do robô, parando. Obstáculo mais próximo a {min(r for r in front_ranges if r < self.safety_distance)} metros.")
        elif not self.back_clear and self.linear_speed < 0:
            self.stop_robot()
            print(f"Obstáculo detectado perto do robô ao reverter, parando. Obstáculo mais próximo a {min(r for r in back_ranges if r < self.safety_distance)} metros.")
        # else:
            #print("Caminho está livre.")


    def connect(self):
        if not self.connected:
            self.connected = True
            print("Robô conectado e pronto para publicar.")

    def disconnect(self):
        if self.connected:
            self.connected = False
            print("Robô desconectado, por favor reconecte para usar.")

    def move_robot(self):
        if self.connected:
            msg = Twist()
            msg.linear.x = self.linear_speed
            msg.angular.z = self.angular_speed
            self.publisher.publish(msg)
            print(f"Movendo: velocidade linear={self.linear_speed} m/s, velocidade angular={self.angular_speed} rad/s")

    def stop_robot(self):
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.move_robot()
        print("Parando robô.")

    def increase_linear_speed(self):
        if self.front_clear:
            self.linear_speed += 0.1
            self.move_robot()
        else:
            self.stop_robot()
            print("Movimento bloqueado na frente! Incapaz de mover para frente.")

    def decrease_linear_speed(self):
        if self.back_clear:
            self.linear_speed -= 0.1
            self.move_robot()
        else:
            self.stop_robot()
            print("Movimento bloqueado atrás! Incapaz de mover para trás.")

    def increase_angular_speed(self):
        self.angular_speed += 0.1
        self.move_robot()

    def decrease_angular_speed(self):
        self.angular_speed -= 0.1
        self.move_robot()

    def kill_switch(self):
        self.killed = True
        self.stop_robot()

    def start_switch(self):
        self.killed = False

def get_key(settings):
    if os.name == 'nt':
        if msvcrt.kbhit():
            return msvcrt.getch().decode()
        return ''
    else:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def teleop_mode(robot_controller):
    settings = None  
    try:
        if os.name != 'nt':
            settings = termios.tcgetattr(sys.stdin)
        print("Entrando em modo de teleoperação. Use as seguintes teclas para controlar o robô:")
        print("  _______ ")
        print(" |   w   |")
        print(" | a s d |")
        print(" |_______|")
        print(" Use 'w', 's', 'a', 'd' para movimento.")
        print(" Use 'espaço' para parar.")
        print(" Use 'q' para sair.")
        print(" Use 'b' para acionar o interruptor de emergência e parar.")
        while True:
            key = get_key(settings)  
            if key == 'w':
                robot_controller.increase_linear_speed()
            elif key == 's':
                robot_controller.decrease_linear_speed()
            elif key == 'a':
                robot_controller.increase_angular_speed()
            elif key == 'd':
                robot_controller.decrease_angular_speed()
            elif key == ' ':
                robot_controller.stop_robot()
            elif key == 'b':
                robot_controller.kill_switch()
                break
            elif key == 'q':
                break
            time.sleep(0.1)  
    finally:
        if os.name != 'nt' and settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def user_interaction(robot_controller):
    questions = [
        inquirer.List('action',
                    message="Que ação você gostaria de realizar? (Nota: você deve conectar antes de teleoperar)",
                    choices=['Teleoperar', 'Conectar', 'Desconectar', "Parada de Emergência",'Sair'])
    ]
    
    while True:
        answers = inquirer.prompt(questions)
        action = answers['action']
        if action == 'Teleoperar':
            teleop_mode(robot_controller)
        elif action == 'Conectar':
            robot_controller.connect()
        elif action == 'Desconectar':
            robot_controller.disconnect()
        elif action == 'Parada de Emergência':
            robot_controller.kill_switch()
        elif action == 'Sair':
            break  # Sair do programa
    robot_controller.disconnect()
    rclpy.shutdown()
    exit(1)

@app.command()
def main():
    rclpy.init()
    robot_controller = RobotController()
    user_thread = threading.Thread(target=user_interaction, args=(robot_controller,))
    user_thread.start()

    try:
        rclpy.spin(robot_controller)
        user_thread.join()
    except KeyboardInterrupt:
        robot_controller.disconnect()
        rclpy.shutdown()
        user_thread.join()

if __name__ == '__main__':
    app()
