from std_msgs.msg import Float64MultiArray
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import numpy as np

def main(args=None):
    rclpy.init(args=args)
    node = Node('send_float64_multiarray')
    # best-effort, volatile, keep last
    qos = qos_profile_sensor_data
    pub = node.create_publisher(Float64MultiArray, '/velocity_controller/commands', qos)
    msg = Float64MultiArray()
    msg.data = [0.0, 0.0]
    i = 0
    print("Publishing to /velocity_controller/commands")
    while rclpy.ok():
        msg.data[0] = i
        msg.data[1] = i
        i += 0.1
        if i > 2*np.pi:
            i = 0
        pub.publish(msg)
        node.get_logger().info('Publishing: "%s"' % msg.data)
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()