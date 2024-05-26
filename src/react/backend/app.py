# OBS: este codigo ficara no robo

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        if not self.cap.isOpened():
            self.get_logger().error("Erro: Não foi possível acessar a webcam.")
            return
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        timer_period = 0.033  # 30 FPS
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
            self.get_logger().info('Publicando imagem da câmera')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber_ = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.emergency_client = self.create_client(Empty, 'emergency_stop')
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

    def move_robot(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.publisher_.publish(msg)
        self.get_logger().info(f'Movendo: velocidade linear={self.linear_speed} m/s, velocidade angular={self.angular_speed} rad/s')

    def stop_robot(self):
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.move_robot()
        self.get_logger().info('Parando o robô.')

    def increase_linear_speed(self):
        if self.front_clear:
            self.linear_speed = 0.1
            self.move_robot()
        else:
            self.stop_robot()

    def decrease_linear_speed(self):
        if self.back_clear:
            self.linear_speed = -0.1
            self.move_robot()
        else:
            self.stop_robot()

    def increase_angular_speed(self):
        self.angular_speed = 0.4
        self.move_robot()

    def decrease_angular_speed(self):
        self.angular_speed = -0.4
        self.move_robot()

    def send_emergency_stop(self):
        if self.emergency_client.wait_for_service(timeout_sec=1.0):
            req = Empty.Request()
            future = self.emergency_client.call_async(req)
            future.add_done_callback(self.emergency_stop_callback)
        else:
            self.get_logger().error('Serviço de emergência não disponível, DESLIGUE O ROBÔ')

    def emergency_stop_callback(self, future):
        try:
            future.result()
            self.get_logger().info('Sinal de parada de emergência enviado com sucesso, processo do robô terminado.')
        except Exception as e:
            self.get_logger().error(f'Falha ao chamar o serviço de parada de emergência: {e}, RETIRE A BATERIA DO ROBÔ!!!')

    def kill_switch(self):
        self.get_logger().info("Parada de emergência forçada do processo.")
        self.stop_robot()
        self.send_emergency_stop()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    robot_controller = RobotController()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(image_publisher)
    executor.add_node(robot_controller)
    try:
        executor.spin()
    finally:
        image_publisher.destroy_node()
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
