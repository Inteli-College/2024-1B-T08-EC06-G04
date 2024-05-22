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
        self.safety_distance = 0.35  # Reduced the stopping distance to half
        self.front_clear = True
        self.back_clear = True

    def lidar_callback(self, msg):
        num_ranges = len(msg.ranges)
        sector_size = num_ranges // 12  # Adjusted to make the cone narrower

        # Define sector indices using the division logic
        front_left_indices = range(num_ranges - sector_size, num_ranges)  # Front left part
        front_right_indices = range(0, sector_size)  # Front right part
        back_indices = range(5 * sector_size, 7 * sector_size)  # Back part

        front_ranges = []
        back_ranges = []

        # Collect front left ranges
        for index in front_left_indices:
            if 0.01 < msg.ranges[index] < 100.0:  # Ensure range is within valid bounds
                front_ranges.append(msg.ranges[index])

        # Collect front right ranges
        for index in front_right_indices:
            if 0.01 < msg.ranges[index] < 100.0:  # Ensure range is within valid bounds
                front_ranges.append(msg.ranges[index])

        # Collect back ranges
        for index in back_indices:
            if 0.01 < msg.ranges[index] < 100.0:  # Ensure range is within valid bounds
                back_ranges.append(msg.ranges[index])

        # Check if any of the distances in the front or back ranges are below the safety threshold
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
            print(f"Obstacle detected close to robot, stopping. Closest obstacle at {min(r for r in front_ranges if r < self.safety_distance)} meters.")
        elif not self.back_clear and self.linear_speed < 0:
            self.stop_robot()
            print(f"Obstacle detected close to robot while reversing, stopping. Closest obstacle at {min(r for r in back_ranges if r < self.safety_distance)} meters.")
        # else:
            #print("Path is clear.")


    def connect(self):
        if not self.connected:
            self.connected = True
            print("Robot connected and ready to publish.")

    def disconnect(self):
        if self.connected:
            self.connected = False
            print("Robot disconnected, please reconnect to use.")

    def move_robot(self):
        if self.connected:
            msg = Twist()
            msg.linear.x = self.linear_speed
            msg.angular.z = self.angular_speed
            self.publisher.publish(msg)
            print(f"Moving: linear speed={self.linear_speed} m/s, angular speed={self.angular_speed} rad/s")

    def stop_robot(self):
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.move_robot()
        print("Stopping robot.")

    def increase_linear_speed(self):
        if self.front_clear:
            self.linear_speed += 0.1
            self.move_robot()
        else:
            self.stop_robot()
            print("Movement blocked in front! Unable to move forward.")

    def decrease_linear_speed(self):
        if self.back_clear:
            self.linear_speed -= 0.1
            self.move_robot()
        else:
            self.stop_robot()
            print("Movement blocked in back! Unable to move backward.")

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
        print("Entering teleoperation mode. Use the following keys to control the robot:")
        print("  _______ ")
        print(" |   w   |")
        print(" | a s d |")
        print(" |_______|")
        print(" Use 'w', 's', 'a', 'd' for movement.")
        print(" Use 'space' to stop.")
        print(" Use 'q' to quit.")
        print(" Use 'b' to trigger the kill switch and stop.")
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
                    message="What action would you like to perform? (Note: you must connect before teleoperating)",
                    choices=['Teleoperate', 'Connect', 'Disconnect', "Emergency Stop",'Exit'])
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
        elif action == 'Emergency Stop':
            robot_controller.kill_switch()
        elif action == 'Exit':
            break  # Exit the program
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