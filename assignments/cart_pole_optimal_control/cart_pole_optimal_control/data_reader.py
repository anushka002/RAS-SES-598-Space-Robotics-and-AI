import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock
import csv
import os

class DataReaderNode(Node):
    def __init__(self):
        super().__init__('data_reader')
        
        # Subscribers
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.create_subscription(Float64, '/model/cart_pole/joint/cart_to_base/cmd_force', self.cart_force_callback, 10)
        self.create_subscription(Float64, '/earthquake_force', self.earthquake_force_callback, 10)
        self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        
        # Data storage
        self.data = {
            "cart_position": 0.0,
            "cart_velocity": 0.0,
            "cart_force": 0.0,
            "earthquake_force": 0.0,
            "pole_angle": 0.0,
            "pole_angular_velocity": 0.0,
            "recovery_time": None
        }
        
        # Max deviation and errors
        self.max_pole_angle_deviation = 0.0
        self.max_cart_position_error = 0.0
        self.max_cart_force = 0.0
        self.initial_cart_position = None
        self.initial_pole_angle = None
        
        # Recovery time tracking
        self.deviation_start_time = None
        self.current_time = 0.0
        
        # File path
        self.file_path = os.path.join('cart_pole_optimal_control/ros2_data_log.csv')
        os.makedirs(os.path.dirname(self.file_path), exist_ok=True)
        
        # Initialize CSV file with headers
        with open(self.file_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["cart_position", "pole_angle", "cart_force", "earthquake_force", 
                             "max_pole_angle_deviation", "max_cart_position_error", "max_cart_force", "recovery_time"])
        
        # Timer to write data periodically
        self.create_timer(1.0, self.write_to_file)
    
    def clock_callback(self, msg):
        # Update current simulation time from /clock topic
        self.current_time = msg.clock.sec + msg.clock.nanosec / 1e9
    
    def joint_states_callback(self, msg):
        try:
            cart_index = msg.name.index('cart_to_base')
            pole_index = msg.name.index('pole_joint')
            
            # Current state data
            cart_position = msg.position[cart_index]
            pole_angle = msg.position[pole_index] * 180  # degrees

            # Record the initial position
            if self.initial_cart_position is None:
                self.initial_cart_position = cart_position
            if self.initial_pole_angle is None:
                self.initial_pole_angle = pole_angle

            # Calculate cart position error
            cart_position_error = abs(cart_position - self.initial_cart_position)
            self.max_cart_position_error = max(self.max_cart_position_error, cart_position_error)

            # Calculate pole angle deviation
            pole_angle_deviation = abs(pole_angle - self.initial_pole_angle)
            self.max_pole_angle_deviation = max(self.max_pole_angle_deviation, pole_angle_deviation)

            # Store current values
            self.data["cart_position"] = cart_position
            self.data["pole_angle"] = pole_angle
            
            # Track recovery time for pole angle using /clock
            if self.deviation_start_time is None:
                if pole_angle_deviation > 10.0:  # Threshold for deviation
                    self.deviation_start_time = self.current_time
            elif pole_angle_deviation < 1.0:  # When recovery happens
                self.data["recovery_time"] = self.current_time - self.deviation_start_time
                self.deviation_start_time = None
            
        except ValueError:
            pass
    
    def cart_force_callback(self, msg):
        self.data["cart_force"] = msg.data  # Newtons
        
        # Track maximum control force
        self.max_cart_force = max(self.max_cart_force, abs(self.data["cart_force"]))
    
    def earthquake_force_callback(self, msg):
        self.data["earthquake_force"] = msg.data  # Newtons
    
    def write_to_file(self):
        # Log all data periodically
        self.data["max_pole_angle_deviation"] = self.max_pole_angle_deviation
        self.data["max_cart_position_error"] = self.max_cart_position_error
        self.data["max_cart_force"] = self.max_cart_force
        
        with open(self.file_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                self.data["cart_position"], self.data["pole_angle"], self.data["cart_force"], self.data["earthquake_force"],
                self.max_pole_angle_deviation, self.max_cart_position_error, self.max_cart_force, self.data["recovery_time"]
            ])
        
        self.get_logger().info(f'Data logged: {self.data}')
        self.get_logger().info("---------------------------------")

def main(args=None):
    rclpy.init(args=args)
    node = DataReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
