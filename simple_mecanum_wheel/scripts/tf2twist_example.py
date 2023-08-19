import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState

import math
from tf2_ros import TransformBroadcaster, TransformStamped, TransformListener, Buffer, LookupException, ExtrapolationException, ConnectivityException

# import tf2_ros

import geometry_msgs

class tf2_twist:

    def __init__(self):

        _node = Node("tf2twist")

        qos_profile = QoSProfile(depth=10)
        # loop_rate = _node.create_rate(100)
      
        tfBuffer = Buffer()
        listener = TransformListener(tfBuffer,_node, qos=qos_profile)

        twist_data = geometry_msgs.msg.Twist()

        pub_cmd_vel = _node.create_publisher(geometry_msgs.msg.Twist, "cmd_vel", qos_profile)
        
        # self.transfromstamped = 
        try:
            while rclpy.ok():
                rclpy.spin_once(_node)

                now = _node.get_clock().now() - rclpy.duration.Duration(seconds=0,nanoseconds=1000000)
                try:
                    trans = tfBuffer.lookup_transform('odom_frame', 'base_link', now, rclpy.duration.Duration(seconds=0, nanoseconds= 0))
                    # print(trans)
                    # print(trans.transform.rotation.x)

                    roll, pitch, yaw = self.quaternion_to_euler(trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)

                    # print(yaw)
                    if(yaw> 0.3):
                        twist_data.angular.z = 70.0#yaw*125*1.3
                    elif(yaw< -0.3):
                        twist_data.angular.z = -70.0#yaw*125*1.3
                    else:
                        twist_data.angular.z = 0.0
                    pub_cmd_vel.publish(twist_data)


                    
                except (LookupException, LookupError, ConnectionAbortedError, ConnectionError, ConnectionRefusedError, ConnectionResetError,ExtrapolationException,ConnectivityException) as e:
                    # print(e)
                    pass
                
        except (KeyboardInterrupt):
                pass
    
    def quaternion_to_euler(self, q0, q1, q2, q3):
        q0q0 = q0 * q0
        q0q1 = q0 * q1
        q0q2 = q0 * q2
        q0q3 = q0 * q3
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q3q3 = q3 * q3

        a = math.atan2(2.0 * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3)
        b = math.asin(2.0 * (q0q2 - q1q3))
        c = math.atan2(2.0 * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3)

        return a , b, c


# def quaternion_to_euler(q):

def ros_main(args = None):
    rclpy.init(args=args)

    tf2twist_class = tf2_twist()
    rclpy.spin(tf2twist_class)

    tf2twist_class.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    ros_main()