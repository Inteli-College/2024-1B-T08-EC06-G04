import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

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

def main():
    rclpy.init()
    robot_controller = RobotController()

    try:
        while not robot_controller.shutdown_requested:
            print("Available actions: Move, Stop, Connect, Disconnect, Exit")
            action = input("What action do you want to perform? ")

            if action == 'Move':
                x = input("Enter X position (linear speed): ")
                angular_theta = input("Enter angular theta position (angular speed): ")
                move_data = {'x': x, 'angular_theta': angular_theta}
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
    main()
