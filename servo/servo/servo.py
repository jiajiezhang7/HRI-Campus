import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class SerialCommunication(Node):
    def __init__(self):
        super().__init__('serial_communication')
        # 打开串口，确保设备连接的端口和波特率正确
        self.serial_port = serial.Serial("/dev/ttyACM0", 9600, timeout=2)
        
        # 订阅 /face_angle 话题
        self.subscription = self.create_subscription(
            Float32,
            '/face_angle',
            self.face_angle_callback,
            10)
        
        self.angle = 0.0  # 默认角度值
        self.subscription  # 防止未使用的变量被垃圾回收

    def face_angle_callback(self, msg):
        # 当接收到 /face_angle 话题的数据时更新角度值
        self.angle = msg.data / 2
        # self.get_logger().info(f'Received face angle: "{self.angle}"')
        
        # 将接收到的角度发送到串口
        self.send_to_serial(self.angle)
    
    def send_to_serial(self, angle):
        # 将接收到的角度值发送到串口
        self.serial_port.write((str(angle) + '\n').encode('ascii'))  # 确保发送换行符以便Arduino端正确解析
        # self.get_logger().info(f'Sent to serial: "{angle}"')

        # 读取串口数据
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('ascii').strip()
            # self.get_logger().info(f'Received from serial: "{line}"')

def main(args=None):
    rclpy.init(args=args)
    serial_communication = SerialCommunication()
    rclpy.spin(serial_communication)
    serial_communication.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()