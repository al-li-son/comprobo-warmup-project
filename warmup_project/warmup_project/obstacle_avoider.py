"""Avoid obstacles with the Neato"""

from cmath import nan
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from neato2_interfaces.msg import Bump
from geometry_msgs.msg import Twist # To control Neato motors
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from visualization_msgs.msg import Marker
from scipy.stats import norm
import matplotlib.pyplot as plt

class ObstacleAvoiderNode(Node):
    def __init__(self):
        super().__init__('person_follower_node')

        self.error = 0.0
        self.x_avg = 0.0
        self.y_avg = 0.0
        self.collision = False

        # Create timer
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.drive_msg)

        # Create publishers/subscribers
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, 'scan', self.get_turn_direction, 10)
        self.bump_subscriber = self.create_subscription(Bump, 'bump', self.hit_obstacle, 10)

        # Make Kp adjustable through ROS args
        self.declare_parameters(namespace='',
        parameters=[('Kp', 0.1)])
        self.Kp = self.get_parameter('Kp').value
        # the following line is only need to support dynamic_reconfigure
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        """ This callback allows the parameters of the node to be adjusted
            dynamically by other ROS nodes (e.g., dynamic_reconfigure).
            This function is only needed to support dynamic_reconfigure. """
        for param in params:
            if param.name == 'Kp' and param.type_ == Parameter.Type.DOUBLE:
                self.Kp = param.value
        print(self.Kp)
        return SetParametersResult(successful=True)
    
    def hit_obstacle(self, msg):
        """
        Check if the Neato has collided with an obstacle
        """       
        self.collision = msg.left_front or msg.left_side or msg.right_front or msg.right_side
    
    def drive_msg(self):
        """
        Publish linear and angular velocities to robot
        """
        move_msg = Twist()
        # Stop moving if the robot has hit an obstacle
        if self.collision:
            move_msg.linear.x = 0.0
            move_msg.angular.z = 0.0
        # Move forward constantly, turn based on calculated error to ideal heading
        else:
            move_msg.linear.x = 0.2
            if self.error > 2:
                move_msg.angular.z = self.Kp * (self.error + 8)
            elif self.error < -2:
                move_msg.angular.z = self.Kp * (self.error - 8)
            else:
                move_msg.angular.z = self.Kp * self.error
        
        self.publisher.publish(move_msg)

    def get_turn_direction(self, msg):
        """
        Determine most favorable robot heading direction based on LIDAR scans.
        Most favorable is the heading with the clearest path forward (no obstacles).
        """
        # Get scans and replace inf with max range (5)
        scans = np.array([val if (not np.isinf(val)) and val > 0.0 else np.nan for val in msg.ranges])
        # Create a normal curve with 181 points (for 0-180 degrees), going out to 3 sigma
        angles = np.linspace(-3,3,181)
        weights = norm.pdf(angles, loc=0, scale=1)
        # Get the scans 180 degrees in front of the robot (90-0 + 359-270 in Neato coordinate system)
        scans_cropped = np.concatenate((np.flip(scans[0:91]), np.flip(scans[270:360])))
        # Scale data by dividing by max
        scans_max = np.nanmax(scans_cropped)
        print(f"{scans_max=}")
        scans_normalized= [val/scans_max if not np.isnan(val) else 1.0 for val in scans_cropped]
        print(f"{scans_normalized}")
        scans_normalized = np.concatenate(([scans_normalized[0], scans_normalized[0]], scans_normalized, [scans_normalized[-1], scans_normalized[-1]]))
        scans_smoothed = []
        for i in range(2, len(scans_normalized)-2):
            scans_smoothed.append(np.mean(scans_normalized[i-2:i+2]))
        # Add scans data to normal curve to determine most favorable heading angle (index of array max)
        scans_scaled = weights + np.array(scans_smoothed)
        turn_angle = np.argmax(scans_scaled)
        # 90 is target heading
        self.error = 90 - turn_angle


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoiderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()