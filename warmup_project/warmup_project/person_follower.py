"""Follow a perosn wiht the Neato"""

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from neato2_interfaces.msg import Bump
from geometry_msgs.msg import Twist # To control Neato motors
from statistics import mean
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from visualization_msgs.msg import Marker

class PersonFollowerNode(Node):
    def __init__(self):
        super().__init__('person_follower_node')

        self.wall_error = 0.0
        self.x_avg = 0.0
        self.y_avg = 0.0
        self.collision = False
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.drive_msg)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.vis_pub = self.create_publisher(Marker, 'marker', 10)
        self.subscriber = self.create_subscription(LaserScan, 'scan', self.get_wall_error, 10)
        self.bump_subscriber = self.create_subscription(Bump, 'bump', self.hit_obstacle, 10)
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
        #if not self.collision:
            self.collision = msg.left_front or msg.left_side or msg.right_front or msg.right_side
    
    def drive_msg(self):
        move_msg = Twist()
        if self.collision:
            move_msg.linear.x = 0.0
            move_msg.angular.z = 0.0
        else:
            move_msg.linear.x = .2
            move_msg.angular.z = self.Kp * self.wall_error
        
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        #marker.ns = "my_namespace"
        marker.id = 0

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.x_avg
        print(f"{self.x_avg=}")
        marker.pose.position.y = self.y_avg
        print(f"{self.y_avg=}")
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0 # Don't forget to set the alpha!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        self.publisher.publish(move_msg)
        self.vis_pub.publish(marker)

    def get_wall_error(self, msg):
        scans = np.array(msg.ranges)
        ranges = np.array(range(361))
        x = 0

        while x < len(scans):
            if scans[x] > 1.5 or scans[x] <= 0:
                scans = np.delete(scans,x)
                ranges = np.delete(ranges,x)
            else:
                x += 1

        if len(scans) > 0:
            x_coords = np.multiply(scans, np.cos(np.multiply(ranges,(np.pi/180))))
            y_coords = np.multiply(scans, np.sin(np.multiply(ranges,(np.pi/180))))
            self.x_avg = np.average(x_coords)
            self.y_avg = np.average(y_coords)

            rho = np.sqrt(self.x_avg**2 + self.y_avg**2)
            phi = np.arctan2(self.y_avg, self.x_avg)

            if phi <= 180:
                self.wall_error = phi
            else:
                self.wall_error = -(360-phi)
        else:
            self.wall_error = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()