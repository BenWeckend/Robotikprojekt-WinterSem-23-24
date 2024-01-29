import rclpy
import rclpy.node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class SimpleDriving(rclpy.node.Node):

    def __init__(self):
        super().__init__('drive_with_scanner')

        # definition of the parameters that can be changed at runtime
        self.declare_parameter('distance_to_stop', 0.3)
        self.declare_parameter('speed_turn', 0.4)
        self.declare_parameter('speed_drive', 0.18)

        # variable for the last sensor reading
        self.min_index = 0
        self.min_value = 0.0
        self.last_scan_msg = None

        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # create subscribers for laser scan data with changed qos
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        # create publisher for driving commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

        # create timer to periodically invoke the driving logic
        timer_period = 0.1 
        self.my_timer = self.create_timer(timer_period, self.timer_callback)





    def scanner_callback(self, msg):
            laser_data = msg.ranges

            self.min_value = float('inf') 
            self.min_index = None
            #finding the smallest value except from 0.0, but still counting the index
            for i, x in enumerate(laser_data):
                if x != 0.0 and x < self.min_value:
                    self.min_value = x
                    self.min_index = i
    
    # driving logic
    def timer_callback(self):
        speed_drive = self.get_parameter('speed_drive').get_parameter_value().double_value
        distance_stop = self.get_parameter('distance_to_stop').get_parameter_value().double_value
        turn_to = self.get_parameter('speed_turn').get_parameter_value().double_value

        speed = 0.0
        turn = 0.0

        #Moving and stopping conditions:
        if self.min_value < distance_stop:
            turn = 0.0
            speed = 0.0
            print('Stopping: Closest object is too close')
        elif self.min_index > 5 and self.min_index < 180:
            speed = 0.0
            turn = turn_to
            print("Index: ", self.min_index,  "  Value: ", self.min_value, 'Turn left')
        elif self.min_index < 355 and self.min_index > 180:
            speed = 0.0
            turn = -turn_to
            print("Index: ", self.min_index,  "  Value: ", self.min_value, 'Turn right')
        else:
            speed = speed_drive
            turn = 0.0
            print("Index: ", self.min_index,  "  Value: ", self.min_value, 'Moving')

        
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = turn
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = SimpleDriving()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()