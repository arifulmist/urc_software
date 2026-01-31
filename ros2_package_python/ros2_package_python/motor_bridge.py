import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from teensy_serial_backend import TeensyComms

class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.teensy = TeensyComms()
        self.teensy.setup('/dev/ttyACM0', 115200, 500)
    def map_range(value, l1, r1, l2, r2):
        return l2 + (value - l1) * (r2 - l2) / (r1 - l1)
    def listener_callback(self, msg):
       
        wheel_base = 0.07  # meters
        wheel_radius = 0.30 # meters
        v = msg.linear.x
        omega = msg.angular.z
        left = float((v - omega * wheel_base / 2))
        right = float((v + omega * wheel_base / 2))

        # print(v, omega, left, right)
        left = int(MotorBridge.map_range(left, -1.0, 1.0, 1000, 2000))
        right = int(MotorBridge.map_range(right, -1.0, 1.0, 1000, 2000))
        cmd = f"[{left},{right}]"
        self.teensy.send_msg(cmd)
        response = self.teensy.send_msg(cmd)
        print(f"Sent: {cmd.strip()} | Teensy responded: {response.strip()}")

        

def main(args=None):
    rclpy.init(args=args)
    node = MotorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()