import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Wrench

from allocation import Allocation
import numpy as np

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('allocation_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        self.subscriber_ = self.create_subscription(Wrench, 'desired_wrench', self.qp, 10)

        self.iterations = 30
        self.solver = Allocation()
        

    def qp(self, msg: Wrench):
        force_d = msg.force
        torque_d = msg.torque
        self.get_logger().info(f'Got Force: {force_d} and Torque: {torque_d}')

        tau = np.array([[force_d.x, force_d.y, torque_d.z]])

        force, angle = self.solver.solve_thrust(self.iterations, tau)






def main(args=None):
    rclpy.init(args=args)

    allocation_node = MinimalPublisher()

    rclpy.spin(allocation_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    allocation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()