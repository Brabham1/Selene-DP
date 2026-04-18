import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import JointState

from thrust_alloc.thrust_allocation import ThrustAlloc
import numpy as np

class Allocation_node(Node):

    def __init__(self):
        super().__init__('allocation_node')
        self.pub_angle = self.create_publisher(JointState, '/selene/servos', 10)
        self.pub_force = self.create_publisher(Float64MultiArray, '/selene/thrusters', 10)

        self.subscriber_ = self.create_subscription(Wrench, 'desired_wrench', self.qp, 10)

        self.iterations = 30
        self.solver = ThrustAlloc()

    def qp(self, msg: Wrench):
        force_d = msg.force
        torque_d = msg.torque
        self.get_logger().info(f'Got Force: {force_d} and Torque: {torque_d}')

        tau = np.array([force_d.x, force_d.y, torque_d.z])

        force, angle = self.solver.solve_thrust(self.iterations, tau)

        self.get_logger().info(f'Calc force: {force}, angle: {np.rad2deg(angle)}')

        angle_msg = JointState()
        angle_msg.header.stamp = self.get_clock().now().to_msg()
        angle_msg.name = ["selene/Joint1", "selene/Joint2", "selene/Joint3", "selene/Joint4"]
        angle_msg.position = [angle[0], angle[1], angle[2], angle[3]]

        self.pub_angle.publish(angle_msg)

        force_msg = Float64MultiArray()
        force_msg.data = [force[0], force[1], force[2], force[3]]

        self.pub_force.publish(force_msg)

def main(args=None):
    rclpy.init(args=args)

    allocation_node = Allocation_node()

    rclpy.spin(allocation_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    allocation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()