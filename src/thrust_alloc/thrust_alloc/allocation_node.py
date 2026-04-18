import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Wrench

import osqp
import numpy as np
from numpy import sin, cos

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.subscriber_ = self.create_subscription(Wrench, 'desired_wrench', self.quadratic_programming, 10)

        self.thrust_count = 4
        self.solver = osqp.OSQP()

        self.tau = np.zeros(3)

        self.f_0 = np.zeros(4)
        self.f = np.zeros(4)
        self.f_min = -40
        self.f_max = 51.52

        self.a_0 = np.zeros(4)
        self.a = np.zeros(4)
        self.a_min = -np.pi
        self.a_max = np.pi
        self.da_max = 2

        self.l_1 = 0.63
        self.l_2 = -0.63
        self.theta = np.deg2rad(41.1)

        # For calculating g
        self.rho = 1000.0
        self.epsi = 1e-06
        self.W_inv = np.eye(4)
        self.h = 1e-6 #Step for calculating gradient (derivative)
     


        self.P = np.diag(self.thrust_count)
        self.Q = np.diag(self.thrust_count)
        self.omega = np.diag(self.thrust_count)
        self.P_pqo = 2 * np.array([[self.P,0,0],
                               [0,self.Q,0],
                               [0,0,self.omega]])
        
    def solve_thrust(self):
        P = self.P_pqo

        A, l, u = self.calc_constraints(self.tau, self.f_0, self.a_0)

        q = np.array([[2 * (self.f.transpose() @ self.P), self.q(self.a_0)]])

        self.solver.setup(P, q, A, l, u, alpha=1.0),


        self.f = self.solver.solve()

    def calc_constraints(self, tau, f_0, a_0):
        aeq = np.hstack([self.T(a_0), self.H(f_0,a_0), np.eye(3)])
        A = np.vstack([aeq, np.eye(11)]) 

        beq = tau - (self.T(a_0) @ f_0)

        box_l = np.hstack([self.f_min - self.f_0,
                      np.maximum(self.a_min - self.a_0, -self.da_max),
                      -np.inf * np.ones(3)])  
        
        box_u = np.hstack([self.f_max - self.f_0,
                      np.minimum(self.a_max - self.a_0, self.da_max),
                      np.inf * np.ones(3)])
        
        l = np.hstack([beq, box_l])
        u = np.hstack([beq, box_u])

        return A, l, u


    def T(self, a):
        #TODO Fix the trig so it isnt so ass.

        l_1 = self.l_1
        l_2 = self.l_2
        theta = self.theta

        T = np.array([[cos(a[0]),cos(a[1]),cos(a[2]),cos(a[3])],
                      [sin(a[0]), sin(a[1]), sin([2]), sin(a[3])],
                      [l_1 * sin(a[0] + theta), 
                       l_1 * sin(a[1] - theta), 
                       l_2 * sin(a[2] - theta), 
                       l_2 * sin(a[3] + theta)]])

        return T

    def H(self, f_0, a_0):
        #TODO Fix the trig so it isnt so ass.

        theta = self.theta
        l_1 = self.l_1
        l_2 = self.l_2

        T_derivated = np.array([[-sin(a_0[0]), -sin(a_0[1]), -sin(a_0[2]), -sin(a_0[3])],
                      [cos(a_0[0]), cos(a_0[1]), cos(a_0[2]), cos(a_0[3])],
                      [l_1 * cos(a_0[0] + theta), l_1 * cos(a_0[1] -theta), l_2 * cos(a_0[2] -theta), l_2 * cos(a_0[3] + theta)]])

        H = T_derivated @ f_0
        return H

    def g(self, a_0):
        n = len(a_0)
        g = np.zeros(n)
        for i in range(n):
            a_0_pluss = a_0.copy()
            a_0_pluss[i] += self.h

            derivative = (self.det_term(a_0_pluss) - self.det_term(a_0)) / self.h
            g[i] = derivative

        return g


    def det_term(self, a):
        det = np.linalg.det(self.T(a) @ self.W_inv @ self.T(a).T)

        g_not_der = self.rho / (self.epsi + det)
        return g_not_der





    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def quadratic_programming(self):
        pass



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()