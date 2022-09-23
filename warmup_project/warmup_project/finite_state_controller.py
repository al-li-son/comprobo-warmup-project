"""Finite state controller with states for person following and spinning"""

import rclpy
import numpy as np
from enum import Enum
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from neato2_interfaces.msg import Bump
from geometry_msgs.msg import Twist # To control Neato motors
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from visualization_msgs.msg import Marker

class State(Enum):
    SPIN = 1
    PERSON_FOLLOW = 2

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        self.state = State.PERSON_FOLLOW
        self.last_state = State.PERSON_FOLLOW

        self.error = 0.0
        self.x_avg = 0.0
        self.y_avg = 0.0
        self.distance = 0.0
        self.collision = False
        self.counter = 0

        # Create timer
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)

        # Create publishers/subscribers
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.get_error, 10)
        self.bump_subscriber = self.create_subscription(Bump, 'bump', self.hit_obstacle, 10)

        # Allow Kp to be adjusted using ROS args
        self.declare_parameters(namespace='',
        parameters=[('Kp_person', 0.7)])
        self.Kp_person = self.get_parameter('Kp_person').value
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
        if not self.collision:
            self.collision = msg.left_front or msg.left_side or msg.right_front or msg.right_side
    
    def run_loop(self):
        """
        Publish linear and angular velocities to robot
        """
        if self.state == State.PERSON_FOLLOW:
            self.person_follow()
        elif self.state == State.SPIN:
            self.spin()

    def person_follow(self):
        move_msg = Twist()
        if self.last_state == State.SPIN:
            move_msg.linear.x = -0.2
            move_msg.angular.x = 0.0
            self.collision = False
            self.last_state = State.PERSON_FOLLOW
            self.publisher.publish(move_msg)  
        elif self.collision:
            self.state = State.SPIN
        else:
            # Turn without moving forward until the person is in front of the robot
            if abs(self.error) > 0.5:
                move_msg.linear.x = 0.0
            else:
                move_msg.linear.x = 0.2
            # Set angular vel based on how far the person is by angle
            move_msg.angular.z = self.Kp_person * self.error
            
            self.publisher.publish(move_msg)   

    def spin(self):
        move_msg = Twist()
        if self.last_state == State.PERSON_FOLLOW:
            self.collision = False
            self.counter = 0
            self.last_state = State.SPIN
            move_msg.linear.x = -0.3
            move_msg.angular.z = 0.0
            self.publisher.publish(move_msg)
        elif self.collision:
            self.state = State.PERSON_FOLLOW
        else:
            if self.counter < 10:
                move_msg.linear.x = -0.3
                move_msg.angular.z = 0.0
                self.counter += 1
            else:
                move_msg.linear.x = 0.0
                move_msg.angular.z = 4.0
            self.publisher.publish(move_msg) 


    def get_error(self, msg):
        if self.state == State.PERSON_FOLLOW:
            self.get_person_error(msg)

    def get_person_error(self, msg):
        scans = np.array(msg.ranges)
        ranges = np.array(range(361))
        x = 0

        # Filter out dropped scans (0) and scans that are too far away
        while x < len(scans):
            if scans[x] > 0.75 or scans[x] <= 0:
                scans = np.delete(scans,x)
                ranges = np.delete(ranges,x)
            else:
                x += 1

        if len(scans) > 0:
            # Convert scans to cartersian
            x_coords = np.multiply(scans, np.cos(np.multiply(ranges,(np.pi/180))))
            y_coords = np.multiply(scans, np.sin(np.multiply(ranges,(np.pi/180))))
            # Average coordinates to find centroid position of person
            self.x_avg = np.average(x_coords)
            self.y_avg = np.average(y_coords)
            # Calculate distance & angle from robot +x to person
            self.distance = np.sqrt(self.x_avg**2 + self.y_avg**2)
            phi = np.arctan2(self.y_avg, self.x_avg)

            self.error = phi
        else:
            self.error = 0.0        

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()