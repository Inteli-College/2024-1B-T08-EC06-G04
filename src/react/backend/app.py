import asyncio
import websockets
import json
from geometry_msgs.msg import Twist
import threading
import rclpy
from rclpy.node import Node

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear_speed = 0.0
        self.angular_speed = 0.0

    def move_robot(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.publisher.publish(msg)

    def stop_robot(self):
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.move_robot()

    def increase_linear_speed(self):
        self.linear_speed = 0.1
        self.move_robot()

    def decrease_linear_speed(self):
        self.linear_speed = -0.1
        self.move_robot()

    def increase_angular_speed(self):
        self.angular_speed = 0.4
        self.move_robot()

    def decrease_angular_speed(self):
        self.angular_speed = -0.4
        self.move_robot()

async def control_robot(websocket, path):
    rclpy.init()
    robot_controller = RobotController()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(robot_controller)

    def robot_control_loop():
        while rclpy.ok():
            rclpy.spin_once(robot_controller, timeout_sec=0.01)

    control_thread = threading.Thread(target=robot_control_loop, daemon=True)
    control_thread.start()

    try:
        async for message in websocket:
            data = json.loads(message)
            action = data.get("action")
            if action == "increase_linear_speed":
                robot_controller.increase_linear_speed()
            elif action == "decrease_linear_speed":
                robot_controller.decrease_linear_speed()
            elif action == "increase_angular_speed":
                robot_controller.increase_angular_speed()
            elif action == "decrease_angular_speed":
                robot_controller.decrease_angular_speed()
            elif action == "stop_robot":
                robot_controller.stop_robot()
    finally:
        rclpy.shutdown()

async def main():
    async with websockets.serve(control_robot, "localhost", 8766):
        await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main())
