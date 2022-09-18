"""Follow the wall with a Neato"""

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from neato2_interfaces.msg import Bump
from geometry_msgs.msg import Twist
from statistics import mean
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        self.wall_error = 0
        self.collision = False

        # Call drive_msg every 0.1s
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.drive_msg)

        # Create publishers/subscribers
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, 'scan', self.get_wall_error, 10)
        self.bump_subscriber = self.create_subscription(Bump, 'bump', self.hit_obstacle, 10)

        # Allow Kp to be adjusted via ROS args
        self.declare_parameters(namespace='',
        parameters=[('Kp', 0.5)])
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
        # Do not switch collision back to false (stop permanently)
        if not self.collision:
            # Check all bump sensors
            self.collision = msg.left_front or msg.left_side or msg.right_front or msg.right_side
    
    def drive_msg(self):
        """
        Publish linear and angular velocities based on parallelism to wall
        """
        msg = Twist()
        # Stop if the robot has hit an obstacle
        if self.collision:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        # Constant linear speed with proportional control on angular velocity
        else:
            msg.linear.x = .2
            msg.angular.z = - self.Kp * self.wall_error
        # Publish to cmd_vel topic
        self.publisher.publish(msg)

    def get_wall_error(self, msg):
        """
        Callback function for subscription to laser scan data
        Calculates error in parallelism to the wall.
        """
        scans = msg.ranges
        # Get scans at 45 degrees +/- 10 degrees on each side for redundancy
        right_front = [val for val in scans[310:320] if val != 0]
        right_back = [val for val in scans[220:230] if val != 0]
        left_front = [val for val in scans[40:50] if val != 0]
        left_back = [val for val in scans[130:140] if val != 0]
        # Get directly left & right scans +/- 5 degrees for redundancy
        right = [val for val in scans[265:275] if val != 0]
        left = [val for val in scans[85:95] if val != 0]

        # Takes mean of points, ignoring 0s (dropped scan points)
        right_front_mean = mean(right_front) if len(right_front) > 0 else 10
        right_back_mean = mean(right_back) if len(right_back) > 0 else 10
        left_front_mean = mean(left_front) if len(left_front) > 0 else 10
        left_back_mean = mean(left_back) if len(left_back) > 0 else 10
        right_mean = mean(right) if len(right) > 0 else 10
        left_mean = mean(left) if len(left) > 0 else 10

        # Calculates difference between front and back (parallelism)
        # Chooses to follow left or right wall based on closest wall
        self.wall_error = right_front_mean - right_back_mean if right_mean < left_mean else left_back_mean - left_front_mean

        print(f"{right_front_mean=}\n{right_back_mean=}\n{left_front_mean=}\n{left_back_mean=}")

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()