import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class BNO055_Integration(Node):
    def __init__(self):
        super().__init__('bno055_integration')
        self.subscription = self.create_subscription(
            Imu,
            '/bno055/imu', 
            self.imu_callback,
            10)
        

        self.prev_time = None
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)

    def imu_callback(self, msg):
        # Tính thời gian delta
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            self.prev_time = current_time
            return

        dt = current_time - self.prev_time
        self.prev_time = current_time

        # Lấy gia tốc tuyến tính đã loại bỏ trọng lực
        acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # Lọc nhiễu: nếu giá trị quá nhỏ thì gán bằng 0
        acc[np.abs(acc) < 0.05] = 0.0

        # Tích phân vận tốc và vị trí
        self.velocity += acc * dt
        self.position += self.velocity * dt

        self.get_logger().info(f"Vel: {self.velocity.round(3)}, Pos: {self.position.round(3)}")

def main(args=None):
    rclpy.init(args=args)
    node = BNO055_Integration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
