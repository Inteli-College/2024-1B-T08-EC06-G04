# Instrucoes para rodar o codigo com os pacotes python:
# 1. python3 -m venv venv
# 2. source venv/bin/activate
# 3. pip install typer inquirer yaspin
# 4. pip show typer | grep Location
# 5. export PYTHONPATH=$PYTHONPATH:/path/to/typer
# 6. colcon build

import rclpy
import typer
from rclpy.node import Node
from geometry_msgs.msg import Twist
import inquirer

app = typer.Typer()

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.move_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.shutdown_requested = False

    def process_action(self, action, data=None):
        print("Processing...")
        velocity_msg = Twist()
        if action == 'Move':
            velocity_msg.linear.x = float(data['x'])
            velocity_msg.angular.z = float(data['angular_theta'])
            self.move_pub.publish(velocity_msg)
        elif action == 'Stop':
            self.move_pub.publish(velocity_msg)  # Publishing zero velocity
        elif action == 'Connect':
            print("ROS node connected.")
        elif action == 'Disconnect':
            print("ROS node disconnected.")
        elif action == 'Exit':
            print("Exiting program...")
            self.shutdown_requested = True  # Set flag to indicate shutdown
        print(f"Action: {action} processed.")

@app.command()
def main():
    rclpy.init()
    robot_controller = RobotController()

    try:
        while not robot_controller.shutdown_requested:
            questions = [
                inquirer.List('action',
                              message="What action do you want to perform?",
                              choices=['Move', 'Stop', 'Connect', 'Disconnect', 'Exit'])
            ]
            answers = inquirer.prompt(questions)
            action = answers['action']

            if action == 'Move':
                move_questions = [
                    inquirer.Text('x', message="Enter X position (linear speed):"),
                    inquirer.Text('angular_theta', message="Enter angular theta (angular speed):")
                ]
                move_data = inquirer.prompt(move_questions)
                robot_controller.process_action(action, move_data)

            elif action in ['Stop', 'Connect', 'Disconnect', 'Exit']:
                robot_controller.process_action(action)
                if action == 'Exit':
                    break

            if not robot_controller.shutdown_requested:
                rclpy.spin_once(robot_controller, timeout_sec=0.1)
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    app()
