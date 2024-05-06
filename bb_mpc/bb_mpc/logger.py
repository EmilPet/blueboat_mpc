import rclpy
import csv
from time import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import GlobalPositionTarget




class LoggerNode(Node):

    def __init__(self):
        super().__init__("logger")
                      
        self.position_log = []
        self.control_input_log = []
        self.reference_pos_log = []

        logging_interval_timer = time()

        # Setting quality of service profile
        qos_profile = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT
        )

        ### SUBSCRIPTIONS ###

                
        self.pos_global_sub_ = self.create_subscription(
            NavSatFix, 
            "/mavros/global_position/global", 
            self.pos_global_callback,
            qos_profile
        )

        self.reference_pos_sub_ = self.create_subscription(
            NavSatFix,
            "/current_waypoint/global_position", 
            self.reference_pos_callback,
            qos_profile
        )
        

        self.control_input_sub_ = self.create_subscription(
            PositionTarget,
            "/mavros/setpoint_raw/local",
            self.control_input_callback,
            qos_profile
        )

        self.SBcontrol_input_sub_ = self.create_subscription(
            GlobalPositionTarget,
            "/mavros/setpoint_raw/global",
            self.SBcontrol_input_callback,
            qos_profile
        )
        self.position_logging_timer = self.create_timer(1.0,lambda: self.save_position_to_file(self.position_log))
        self.control_logging_timer = self.create_timer(1.0,lambda: self.save_control_to_file(self.control_input_log))
        self.reference_pos_logging_timer = self.create_timer(1.0,lambda: self.save_reference_position_to_file(self.reference_pos_log))



    ### CALLBACKS ###
    def pos_global_callback(self,msg:NavSatFix):
        timestamp = msg.header.stamp.sec
        latitude = msg.latitude
        longitude = msg.longitude
        self.position_log.append([timestamp, latitude, longitude])

    def reference_pos_callback(self,msg:NavSatFix):
        timestamp = msg.header.stamp.sec
        latitude = msg.latitude
        longitude = msg.longitude
        self.reference_pos_log.append([timestamp,latitude,longitude])
    
    def control_input_callback(self,msg:PositionTarget):
        timestamp = msg.header.stamp.sec
        v = msg.velocity.x
        omega = msg.yaw_rate
        self.control_input_log.append([timestamp,v,omega])

    def SBcontrol_input_callback(self,msg:GlobalPositionTarget):
        timestamp = msg.header.stamp.sec
        v = msg.velocity.x
        yaw = msg.yaw
        self.control_input_log.append([timestamp,v,yaw])

    def save_position_to_file(self, data:list):
        
        time = []
        latitude = []
        longitude = []

        for item in data:
            time.append(item[0])
            latitude.append(item[1])
            longitude.append(item[2])
        
        file_path = "position.csv"

        with open(file_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Time', 'Latitude', 'Longitude'])
            for i in range(len(time)):
                writer.writerow([time[i], latitude[i], longitude[i]])


    def save_reference_position_to_file(self, data:list):
        
        time = []
        latitude = []
        longitude = []

        for item in data:
            time.append(item[0])
            latitude.append(item[1])
            longitude.append(item[2])
        
        file_path = "reference_position.csv"

        with open(file_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Time', 'Reference Latitude', 'Reference Longitude'])
            for i in range(len(time)):
                writer.writerow([time[i], latitude[i], longitude[i]])
            

    def save_SBcontrol_to_file(self, data:list):
        
        time = []
        linear_velocity = []
        angular_velocity = []

        for item in data:
            time.append(item[0])
            linear_velocity.append(item[1])
            angular_velocity.append(item[2])
        
        file_path = "SBcontrol.csv"

        with open(file_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Time', 'Linear Velocity', 'Yaw'])
            for i in range(len(time)):
                writer.writerow([time[i], linear_velocity[i], angular_velocity[i]])
     

    def save_control_to_file(self, data:list):
        
        time = []
        linear_velocity = []
        angular_velocity = []

        for item in data:
            time.append(item[0])
            linear_velocity.append(item[1])
            angular_velocity.append(item[2])
        
        file_path = "control.csv"

        with open(file_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Time', 'Linear Velocity', 'Angular Velocity'])
            for i in range(len(time)):
                writer.writerow([time[i], linear_velocity[i], angular_velocity[i]])
            
               
   
        




def main(args=None):
    rclpy.init(args=args)

    logger = LoggerNode()
    rclpy.spin(logger)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()