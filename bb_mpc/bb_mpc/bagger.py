import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy
import matplotlib.pyplot as plt
import rosbag2_py
from geometry_msgs.msg import PoseStamped


class LoggerNode(Node):

    def __init__(self):
        super().__init__("logger")
        self.writer = rosbag2_py.SequentialWriter()

                        
        # Setting quality of service profile
        qos_profile = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT
        )

        storage_options = rosbag2_py._storage.StorageOptions(
            uri='my_bag1',
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        topic_info = rosbag2_py._storage.TopicMetadata(
            name='/mavros/local_position/pose',
            type='geometry_msgs/msg/PoseStamped',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)

                
        self.pos_local_sub_ = self.create_subscription(
            PoseStamped, 
            "/mavros/local_position/pose", 
            self.pos_local_callback,
            qos_profile
        )


    
    def pos_local_callback(self, msg):
        self.writer.write(
            '/mavros/local_position/pose',
            serialize_message(msg),
            self.get_clock().now().nanoseconds
            )




def main(args=None):
    rclpy.init(args=args)

    logger = LoggerNode()

    rclpy.spin(logger)

    rclpy.shutdown()

if __name__ == '__main__':
    main()