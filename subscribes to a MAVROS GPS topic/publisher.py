import rclpy
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, QoSReliabilityPolicy##quality and reliability of servic

rclpy.init()

def listener_callback(msg):
    # This function runs every time a message is received
    print(f'latitude: "{str(msg.latitude)}"')


# Creating a custom reliable profile
reliable_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    depth=10
)


node = rclpy.create_node('mavros_subscriber')
subscription = node.create_subscription(
    NavSatFix,
    '/mavros/global_position/global',
    listener_callback,
    reliable_qos
)

rclpy.spin(node)


node.destroy_node()
rclpy.shutdown()