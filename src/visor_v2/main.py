import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import streamlit as st
import threading
import time
import cv2
import base64
import numpy as np
from PIL import Image
import io

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
        self.safety_distance = 0.35
        self.front_clear = True
        self.back_clear = True

    def lidar_callback(self, msg):
        num_ranges = len(msg.ranges)
        sector_size = num_ranges // 12

        front_left_indices = range(num_ranges - sector_size, num_ranges)
        front_right_indices = range(0, sector_size)
        back_indices = range(5 * sector_size, 7 * sector_size)

        front_ranges = [msg.ranges[i] for i in front_left_indices if 0.01 < msg.ranges[i] < 100.0] + \
                       [msg.ranges[i] for i in front_right_indices if 0.01 < msg.ranges[i] < 100.0]
        back_ranges = [msg.ranges[i] for i in back_indices if 0.01 < msg.ranges[i] < 100.0]

        self.front_clear = not any(r < self.safety_distance for r in front_ranges)
        self.back_clear = not any(r < self.safety_distance for r in back_ranges)

        if not self.front_clear and self.linear_speed > 0:
            self.stop_robot()
        elif not self.back_clear and self.linear_speed < 0:
            self.stop_robot()

    def connect(self):
        self.connected = True
        print("Robot connected.")

    def disconnect(self):
        self.connected = False
        print("Robot disconnected.")

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

    def decrease_linear_speed(self):
        if self.back_clear:
            self.linear_speed -= 0.1
            self.move_robot()
        else:
            self.stop_robot()

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

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Configuração do Streamlit
        st.title("Robot Controller via ROS 2 and Streamlit")
        self.frame_holder = st.empty()
        self.latency_placeholder = st.empty()

    def listener_callback(self, msg):
        timestamp, jpg_as_text = msg.data.split('|', 1)
        timestamp = float(timestamp)
        current_time = time.time()
        latency = current_time - timestamp

        jpg_original = base64.b64decode(jpg_as_text)
        jpg_as_np = np.frombuffer(jpg_original, dtype=np.uint8)
        img = cv2.imdecode(jpg_as_np, cv2.IMREAD_COLOR)
        if img is not None:
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(img_rgb)
            img_bytes = io.BytesIO()
            pil_image.save(img_bytes, format="JPEG")
            img_bytes.seek(0)
            self.frame_holder.image(
                img_bytes, caption="Webcam Stream", use_column_width=True
            )
            self.latency_placeholder.text(f"Latency: {latency:.4f} seconds")
        else:
            self.get_logger().error('Could not decode the image')

def main():
    if 'ros_initialized' not in st.session_state:
        rclpy.init()
        st.session_state.ros_initialized = True

        robot_controller = RobotController()
        listener = Listener()

        executor = rclpy.executors.MultiThreadedExecutor()

        executor.add_node(robot_controller)
        executor.add_node(listener)

        def spin_executor():
            executor.spin()

        threading.Thread(target=spin_executor, daemon=True).start()

        st.session_state.robot_controller = robot_controller
    else:
        robot_controller = st.session_state.robot_controller

    st.sidebar.title("Control Panel")

    if st.sidebar.button("Connect"):
        robot_controller.connect()

    if st.sidebar.button("Disconnect"):
        robot_controller.disconnect()

    if st.sidebar.button("Increase Linear Speed"):
        robot_controller.increase_linear_speed()

    if st.sidebar.button("Decrease Linear Speed"):
        robot_controller.decrease_linear_speed()

    if st.sidebar.button("Increase Angular Speed"):
        robot_controller.increase_angular_speed()

    if st.sidebar.button("Decrease Angular Speed"):
        robot_controller.decrease_angular_speed()

    if st.sidebar.button("Stop Robot"):
        robot_controller.stop_robot()

    if st.sidebar.button("Kill Switch"):
        robot_controller.kill_switch()

    if st.sidebar.button("Start Switch"):
        robot_controller.start_switch()

    if st.sidebar.button("Exit"):
        robot_controller.disconnect()
        rclpy.shutdown()
        st.stop()

if __name__ == '__main__':
    main()