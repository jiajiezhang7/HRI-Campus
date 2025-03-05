import rclpy
from rclpy.node import Node
import serial

class SerialCommunication(Node):
    def __init__(self):
        super().__init__('serial_communication')
        # 打开串口，确保设备连接的端口和波特率正确
        self.serial_port = serial.Serial("/dev/ttyACM0", 9600, timeout=2)
        # 定时器用来周期性发送和读取串口数据
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0 # 计数器用于发送不同的角度值

    def timer_callback(self):
        # 将计数器值作为角度发送到串口
        angle = self.counter  # 当前角度值
        self.serial_port.write((str(angle) + '\n').encode('ascii'))  # 确保发送换行符以便Arduino端正确解析
        self.get_logger().info(f'Sent to serial: "{angle}"')

        # 读取串口数据
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('ascii').strip()
            self.get_logger().info(f'Received from serial: "{line}"')

        # 更新计数器
        self.counter += 10
        if self.counter > 180:  # 循环发送0-180度的角度值
            self.counter = 0

def main(args=None):
    rclpy.init(args=args)
    serial_communication = SerialCommunication()
    rclpy.spin(serial_communication)
    serial_communication.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
