import rclpy
import typer
from rclpy.node import Node
from geometry_msgs.msg import Twist
import inquirer
import threading
import sys, os

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

    def connect(self):
        if not self.connected:
            self.connected = True
            print("Connected and ready to publish.")

    def disconnect(self):
        if self.connected:
            self.publisher.destroy()
            self.connected = False
            print("Disconnected and cleaned up.")

    def move_robot(self, linear, angular):
        if self.connected:
            msg = Twist()
            msg.linear.x = linear
            msg.angular.z = angular
            self.publisher.publish(msg)
            print(f"Moving: linear={linear} m/s, angular={angular} rad/s")

    def stop_robot(self):
        self.move_robot(0.0, 0.0)
        print("Robot stopped.")

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
    settings = None  # Initialize settings variable
    try:
        if os.name != 'nt':
            settings = termios.tcgetattr(sys.stdin)
        print("Entering teleoperation mode. Use 'w', 's', 'a', 'd' to control the robot, and ' ' to stop. Press 'q' to quit teleop.")
        while True:
            key = get_key(settings)  # Pass settings to get_key
            if key == 'w':
                robot_controller.move_robot(0.1, 0.0)
            elif key == 's':
                robot_controller.move_robot(-0.1, 0.0)
            elif key == 'a':
                robot_controller.move_robot(0.0, 0.1)
            elif key == 'd':
                robot_controller.move_robot(0.0, -0.1)
            elif key == ' ':
                robot_controller.stop_robot()
            elif key == 'q':
                break  # Exit teleop mode
    finally:
        if os.name != 'nt' and settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def user_interaction(robot_controller):
    questions = [
        inquirer.List('action',
                      message="What action do you want to perform?",
                      choices=['Teleoperate', 'Connect', 'Disconnect', 'Exit'])
    ]
    while True:
        answers = inquirer.prompt(questions)
        action = answers['action']

        if action == 'Teleoperate':
            teleop_mode(robot_controller)
        elif action == 'Connect':
            robot_controller.connect()
        elif action == 'Disconnect':
            robot_controller.disconnect()
        elif action == 'Exit':
            break

@app.command()
def main():
    rclpy.init()
    robot_controller = RobotController()
    user_thread = threading.Thread(target=user_interaction, args=(robot_controller,))
    user_thread.start()

    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.disconnect()
        rclpy.shutdown()
        user_thread.join()

if __name__ == '__main__':
    app()