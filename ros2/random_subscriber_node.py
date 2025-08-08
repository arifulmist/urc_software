import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class RandomSubscriber(Node):
    def __init__(self):
        super().__init__('random_subscriber')
       
        self.subscription = self.create_subscription(
            Float64,
            'random_value',
            self.listener_callback,
            10)
        self.subscription  
        self.get_logger().info("Random Subscriber started, listening to /random_value")

    def listener_callback(self, msg):
        
        random_value = msg.data
        self.get_logger().info(f'Received: {random_value}')

def main(args=None):
    rclpy.init(args=args)
    random_subscriber = RandomSubscriber()

    try:
        rclpy.spin(random_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        random_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

