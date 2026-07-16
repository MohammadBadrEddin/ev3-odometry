# imu_bridge_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import socket
import struct
import threading

# imu_packet_t: uint32 + 6x float = 28 bytes
PACKET_FORMAT = '<I6f'
PACKET_SIZE   = 28

HOST = '0.0.0.0'   # allow all
PORT = 9000        # imu port


class ImuBridgeNode(Node):
    def __init__(self):
        super().__init__('imu_bridge_node')
        self.publisher_ = self.create_publisher(Imu, '/imu/raw', 10)

        # TCP-Server Socket
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind((HOST, PORT))
        self.server_sock.listen(1)
        self.get_logger().info('imu_bridge_node: waiting for STM32 on port ' + str(PORT))

        self.thread = threading.Thread(target=self.tcp_loop, daemon=True)
        self.thread.start()

    def tcp_loop(self):
        while True:
            try:
                conn, addr = self.server_sock.accept()
                self.get_logger().info('STM32 connected: ' + str(addr))
                self.receive_loop(conn)
            except Exception as e:
                self.get_logger().error('TCP accept error: ' + str(e))

    def receive_loop(self, conn):
        buffer = b''
        while True:
            try:
                chunk = conn.recv(256)
                if not chunk:
                    self.get_logger().warn('STM32 disconnected')
                    break
                buffer += chunk

                while len(buffer) >= PACKET_SIZE:
                    packet_raw = buffer[:PACKET_SIZE]
                    buffer = buffer[PACKET_SIZE:]
                    self.parse_and_publish(packet_raw)

            except Exception as e:
                self.get_logger().error('TCP recv error: ' + str(e))
                break

    def parse_and_publish(self, raw):
        timestamp_ms, ax, ay, az, gx, gy, gz = struct.unpack(PACKET_FORMAT, raw)

        msg = Imu()

        # Header
        msg.header.frame_id = 'imu_link'
        # STM32-Timestamp in ROS2 time
        msg.header.stamp.sec     = timestamp_ms // 1000
        msg.header.stamp.nanosec = (timestamp_ms % 1000) * 1_000_000

        # accel [m/s²]
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        # angular vel [rad/s]
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        # Orientation unknown
        msg.orientation_covariance[0] = -1.0

        self.publisher_.publish(msg)

    def destroy_node(self):
        self.server_sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
