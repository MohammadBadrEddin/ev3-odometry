import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from sensor_msgs.msg import BatteryState
import socket
import threading


EV3_IP   = '192.168.0.101'  #EV3 IP
EV3_PORT = 5005


class Ev3BridgeNode(Node):
    def __init__(self):
        super().__init__('ev3_bridge_node')

        # Publishers
        self.pub_encoder  = self.create_publisher(Int32,        '/ev3/encoder_r', 10)
        self.pub_steering = self.create_publisher(Int32,        '/ev3/steering',  10)
        self.pub_battery  = self.create_publisher(BatteryState, '/ev3/battery',   10)

        # Subscriber for motor commands from controller_node
        self.sub_cmd = self.create_subscription(
            String, '/ev3/cmd', self.cmd_callback, 10)

        self.conn   = None
        self.lock   = threading.Lock()

        # TCP receive loop in separate thread
        self.thread = threading.Thread(target=self.tcp_loop, daemon=True)
        self.thread.start()

        self.get_logger().info('ev3_bridge_node started, connecting to EV3...')

    def tcp_loop(self):
        while True:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                sock.connect((EV3_IP, EV3_PORT))
                self.get_logger().info('Connected to EV3 at ' + EV3_IP)

                with self.lock:
                    self.conn = sock

                self.receive_loop(sock)

            except Exception as e:
                self.get_logger().error('TCP error: ' + str(e) + ' retrying in 3s...')
                with self.lock:
                    self.conn = None
                import time
                time.sleep(3)

    def receive_loop(self, sock):
        buf = ''
        while True:
            try:
                chunk = sock.recv(256).decode()
                if not chunk:
                    self.get_logger().warn('EV3 disconnected')
                    break
                buf += chunk

                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    line = line.strip()
                    if line:
                        self.parse_and_publish(line)

            except Exception as e:
                self.get_logger().error('recv error: ' + str(e))
                break

        with self.lock:
            self.conn = None

    def parse_and_publish(self, line):
        # Expected format: "enc_r,enc_s,bat_v"
        parts = line.split(',')
        if len(parts) != 3:
            self.get_logger().warn('Unexpected format: ' + line)
            return

        try:
            enc_r = int(parts[0])
            enc_s = int(parts[1])
            bat_v = float(parts[2])
        except ValueError:
            self.get_logger().warn('Parse error: ' + line)
            return

        now = self.get_clock().now().to_msg()

        # Publish encoder_r
        msg_enc = Int32()
        msg_enc.data = enc_r
        self.pub_encoder.publish(msg_enc)

        # Publish steering
        msg_steer = Int32()
        msg_steer.data = enc_s
        self.pub_steering.publish(msg_steer)

        # Publish battery
        msg_bat = BatteryState()
        msg_bat.header.stamp    = now
        msg_bat.voltage         = bat_v
        msg_bat.present         = True
        self.pub_battery.publish(msg_bat)

    def cmd_callback(self, msg):
        # Forward command string to EV3
        with self.lock:
            if self.conn is not None:
                try:
                    self.conn.sendall((msg.data + '\n').encode())
                except Exception as e:
                    self.get_logger().error('Send error: ' + str(e))
            else:
                self.get_logger().warn('No EV3 connection, command dropped')

    def destroy_node(self):
        with self.lock:
            if self.conn:
                self.conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Ev3BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()