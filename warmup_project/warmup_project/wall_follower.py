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
from visualization_msgs.msg import Marker

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        self.wall_error = 0
        self.collision = False
        self.right_front_mean = 0
        self.right_back_mean = 0
        self.left_front_mean = 0
        self.left_back_mean = 0
        self.right_mean = 0
        self.left_mean = 0

        # Call drive_msg every 0.1s
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.drive_msg)

        # Create publishers/subscribers
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.vis_pub = self.create_publisher(Marker, 'marker', 10)
        self.subscriber = self.create_subscription(LaserScan, 'scan', self.get_wall_error, 10)
        self.bump_subscriber = self.create_subscription(Bump, 'bump', self.hit_obstacle, 10)

        # Allow Kp to be adjusted via ROS args
        self.declare_parameters(namespace='',
        parameters=[('Kp', 0.3)])
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
            msg.angular.z = self.Kp * self.wall_error
        # Publish to cmd_vel topic
        self.publisher.publish(msg)

        # Create a markers in rviz to visualize wall following (3 points, front, center, & back)
        markers = [Marker(), Marker(), Marker()]
        for marker in markers:
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "my_namespace"
            marker.id = 0

            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0 # Don't forget to set the alpha!
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

        if self.right_mean < self.left_mean:
            markers[0].pose.position.x = 0.0
            markers[0].pose.position.y = - float(self.right_mean)
            markers[1].pose.position.x = 1/np.sqrt(2) * self.right_front_mean
            markers[1].pose.position.y = - 1/np.sqrt(2) * self.right_front_mean
            markers[2].pose.position.x = - 1/np.sqrt(2) * self.right_back_mean
            markers[2].pose.position.y = - 1/np.sqrt(2) * self.right_back_mean
        else:
            markers[0].pose.position.x = 0.0
            markers[0].pose.position.y = float(self.left_mean)
            markers[1].pose.position.x = 1/np.sqrt(2) * self.left_front_mean
            markers[1].pose.position.y = 1/np.sqrt(2) * self.left_front_mean
            markers[2].pose.position.x = - 1/np.sqrt(2) * self.left_back_mean
            markers[2].pose.position.y = 1/np.sqrt(2) * self.left_back_mean          
        
        self.vis_pub.publish(markers[0])
        self.vis_pub.publish(markers[1])
        self.vis_pub.publish(markers[2])

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
        self.right_front_mean = mean(right_front) if len(right_front) > 0 else 10
        self.right_back_mean = mean(right_back) if len(right_back) > 0 else 10
        self.left_front_mean = mean(left_front) if len(left_front) > 0 else 10
        self.left_back_mean = mean(left_back) if len(left_back) > 0 else 10
        self.right_mean = mean(right) if len(right) > 0 else 10
        self.left_mean = mean(left) if len(left) > 0 else 10

        # Calculates difference between front and back (parallelism)
        # Chooses to follow left or right wall based on closest wall
        self.wall_error = self.right_back_mean - self.right_front_mean \
            if self.right_mean < self.left_mean else self.left_front_mean - self.left_back_mean

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()