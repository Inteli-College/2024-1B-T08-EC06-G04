import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import base64
import numpy as np
import streamlit as st
from PIL import Image
import io
import time

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
        st.title("Visualização da Webcam via ROS 2")
        self.frame_holder = st.empty()
        self.latency_placeholder = st.empty()

    def listener_callback(self, msg):
        timestamp, jpg_as_text = msg.data.split('|', 1)
        timestamp = float(timestamp)
        current_time = time.time()
        latency = timestamp - current_time

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
    rclpy.init()
    listener = Listener()
    
    def run_ros2_listener():
        rclpy.spin_once(listener, timeout_sec=0.1)

    while True:
        run_ros2_listener()
        time.sleep(0.016)  # Aproximadamente 60 FPS

if __name__ == '__main__':
    main()