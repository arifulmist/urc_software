import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class RandomPublisher(Node):
    def __init__(self):
        super().__init__('random_publisher')
        # Publisher that will publish random values to the '/random_value' topic
        self.publisher = self.create_publisher(Float64, 'random_value', 10)
        # Timer to call the timer_callback every 1/60 seconds (60Hz)
        self.timer = self.create_timer(1/60.0, self.timer_callback)
        self.get_logger().info("Random Publisher started at 60 Hz")

    def timer_callback(self):
        # Generate a random float value between 0.0 and 100.0
        random_value = random.uniform(0.0, 100.0)
        msg = Float64()
        msg.data = random_value
        # Publish the random value to the topic
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {random_value}')

def main(args=None):
    rclpy.init(args=args)
    random_publisher = RandomPublisher()

    try:
        rclpy.spin(random_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        random_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

