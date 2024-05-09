import typer
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
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
        self.connected = False
        self.linear_speed = round(0.0,2)
        self.angular_speed = round(0.0,2)
        self.killed = False

    def connect(self):
        if not self.connected:
            self.connected = True
            print("Robô conectado e pronto para publicar.")

    def disconnect(self):
        if self.connected:
            #self.publisher.destroy()
            self.connected = False
            print("Robô desconectado, para utilizar conecte-o novamente.")

    def move_robot(self):
        if self.connected:
            msg = Twist()
            msg.linear.x = self.linear_speed
            msg.angular.z = self.angular_speed
            self.publisher.publish(msg)
            print(f"Movendo: velocidade linear={round(self.linear_speed,2)} m/s, velocidade angular={round(self.angular_speed,2)} rad/s")

    def stop_robot(self):
        self.linear_speed = round(0.0,2)
        self.angular_speed = round(0.0,2)
        self.move_robot()
        print("Parando robô.")

    def decrease_linear_speed(self):
        self.linear_speed -= round(0.1,2)
        self.move_robot()

    def increase_linear_speed(self):
        self.linear_speed += round(0.1,2)
        self.move_robot()

    def decrease_angular_speed(self):
        self.angular_speed -= round(0.1,2)
        self.move_robot()

    def increase_angular_speed(self):
        self.angular_speed += round(0.1,2)
        self.move_robot()
    
    def start_switch(self):
        self.killed = False
    
    def process_state(self):
        return self.killed

    def kill_switch(self):
        self.killed = True
        self.linear_speed = round(0.0,2)
        self.angular_speed = round(0.0,2)
        self.move_robot()


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
        print("Entrando no modo de teleoperação. Use as seguintes teclas para controlar o robô:")
        print("  _______ ")
        print(" |   w   |")
        print(" | a s d |")
        print(" |_______|")
        print(" Use 'w', 's', 'a', 'd' para mover.")
        print(" Use 'espaço' para parar.")
        print(" Use 'q' para sair.")
        print(" Use 'b' para forçar a morte do processo e parada.")
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
                break    # Exit teleop mode
            time.sleep(0.1)  
    finally:
        if os.name != 'nt' and settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def user_interaction(robot_controller):
    questions = [
        inquirer.List('action',
                    message="Qual ação você quer realizar? Obs(para teleoperar, deve-se conectar antes)",
                    choices=['Teleoperar', 'Conectar', 'Desconectar', "Parada de emergência",'Sair'])
    ]
    questions2 = [inquirer.List('action',message="Qual ação será realizada?",choices=['Iniciar processo', 'Sair'])]
    while True:
        # answers = inquirer.prompt(questions)
        # action = answers['action']
        if robot_controller.process_state() == False:
            answers = inquirer.prompt(questions)
            action = answers['action']
            if action == 'Teleoperar':
                teleop_mode(robot_controller)
            elif action == 'Conectar':
                robot_controller.connect()
            elif action == 'Desconectar':
                robot_controller.disconnect()
            elif action == 'Parada de emergência':
                robot_controller.kill_switch()
            elif action == 'Sair':
                break
        else:
            answers = inquirer.prompt(questions2)
            action = answers['action']
            if action == "Iniciar processo":
                robot_controller.start_switch()
            elif action == 'Sair':
                break
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
    #finally:
    #    robot_controller.disconnect()
    #    rclpy.shutdown()
    #    user_thread.join()

if __name__ == '__main__':
    app()