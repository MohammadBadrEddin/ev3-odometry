# controller_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import threading

# Joystick config
DEADZONE     = 3000
STEER_MIN    = 5
STEER_MAX    = 174
STEER_CENTER = 90
STEER_STEP   = 5
DRIVE_MAX    = 60    # max speed percent
LOOP_HZ      = 20   # 50ms loop


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.pub_cmd = self.create_publisher(String, '/ev3/cmd', 10)

        self.steer_pos = STEER_CENTER
        self.running   = True

        self.thread = threading.Thread(target=self.joy_loop, daemon=True)
        self.thread.start()

        self.get_logger().info('controller_node started — waiting for DualSense')

    def publish_cmd(self, cmd_str):
        msg = String()
        msg.data = cmd_str
        self.pub_cmd.publish(msg)
        self.get_logger().debug('CMD: ' + cmd_str)

    def joy_loop(self):
        pygame.init()
        pygame.joystick.init()

        while pygame.joystick.get_count() == 0:
            self.get_logger().warn('No joystick found, retrying...')
            import time
            time.sleep(1)
            pygame.joystick.quit()
            pygame.joystick.init()

        js = pygame.joystick.Joystick(0)
        js.init()
        self.get_logger().info('Joystick connected: ' + js.get_name())

        import time
        clock_interval = 1.0 / LOOP_HZ

        while self.running:
            pygame.event.pump()

            # Steering: axis 0 left stick horizontal (inverted to match vehicle)
            x_raw = int(js.get_axis(0) * 32767)

            if x_raw < -DEADZONE:
                self.steer_pos = min(STEER_MAX, self.steer_pos + STEER_STEP)
                self.publish_cmd('STEER,' + str(self.steer_pos))
            elif x_raw > DEADZONE:
                self.steer_pos = max(STEER_MIN, self.steer_pos - STEER_STEP)
                self.publish_cmd('STEER,' + str(self.steer_pos))

            # Drive: L2 (Axis 2) = forward, R2 (Axis 5) = backward
            # Resting value is -1.0, fully pressed is 1.0
            # Normalize to 0.0 (rest) → 1.0 (full)
            l2 = (js.get_axis(2) + 1.0) / 2.0
            r2 = (js.get_axis(5) + 1.0) / 2.0

            speed = int((l2 - r2) * DRIVE_MAX)

            # Send STOP when no input
            if abs(speed) > 2:
                self.publish_cmd('DRIVE,' + str(speed))
            else:
                self.publish_cmd('STOP')

            # Cross (X) = STOP + center steering
            if js.get_button(0):
                self.steer_pos = STEER_CENTER
                self.publish_cmd('STOP')
                self.publish_cmd('STEER,' + str(STEER_CENTER))

            # L3 = center steering only
            if js.get_button(10):
                self.steer_pos = STEER_CENTER
                self.publish_cmd('STEER,' + str(STEER_CENTER))

            time.sleep(clock_interval)

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()