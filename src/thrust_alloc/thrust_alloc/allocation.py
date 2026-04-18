import osqp
import numpy as np
from numpy import sin, cos
from scipy.linalg import block_diag
from scipy import sparse

class Allocation():
    def __init__(self):
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
        self.da_max = np.deg2rad(2)

        self.l_1 = 0.63
        self.l_2 = -0.63
        self.theta = np.deg2rad(41.1)

        # For calculating g
        self.rho = 1000.0
        self.epsi = 1e-06
        self.W_inv = np.eye(4)
        self.h = 1e-6 #Step for calculating gradient (derivative)
     
        self.P = sparse.eye(self.thrust_count, format='csc')
        self.Q = sparse.eye(3, format='csc')
        self.omega = sparse.eye(self.thrust_count, format='csc')
        self.P_pqo = 2 * sparse.block_diag([self.P, self.omega, self.Q], format='csc')

    def solve_thrust(self, iter, tau):
        self.tau = tau

        for i in range(iter):
            res = self.solve()

            if res and res.info.status == 'solved':
                x = res.x
                diff_f_0 = x[:4]
                diff_a_0 = x[4:8]

                self.f_0 = self.f_0 + diff_f_0
                self.a_0 = self.a_0 + diff_a_0

        if res and res.info.status == 'solved':
            return self.f_0, self.a_0
        
    def solve(self):
        P = self.P_pqo

        self.a_0 = self.wrap_angles(self.a_0)

        A, l, u = self.calc_constraints(self.tau, self.f_0, self.a_0)

        q = np.hstack([2 * (self.f_0 @ self.P.toarray()), self.g(self.a_0), np.zeros(3)])

        self.solver.setup(P, q, A, l, u, alpha=1.0)
        res = self.solver.solve()
        return res
    
    @staticmethod
    def wrap_angles(angle_vec):
        for i in range(len(angle_vec)):
            angle_vec[i] = (angle_vec[i] + np.pi) % (2*np.pi) - np.pi

        return angle_vec

    def calc_constraints(self, tau, f_0, a_0):
        T_sparse = sparse.csc_matrix(self.T(a_0))
        H_sparse = sparse.csc_matrix(self.H(a_0, f_0))
        I_sparse = sparse.eye(3, format='csc')

        aeq = sparse.hstack([T_sparse, H_sparse, I_sparse], format='csc')

        I_box = sparse.eye(11, format='csc')
        A = sparse.vstack([aeq, I_box], format='csc')

        beq = tau - (self.T(a_0) @ f_0)

        box_l = np.hstack([self.f_min - f_0,
                      np.maximum(self.a_min - a_0, -self.da_max),
                      -np.inf * np.ones(3)])  
        
        box_u = np.hstack([self.f_max - f_0,
                      np.minimum(self.a_max - a_0, self.da_max),
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
                      [sin(a[0]), sin(a[1]), sin(a[2]), sin(a[3])],
                      [l_1 * sin(a[0] + theta), 
                       l_1 * sin(a[1] - theta), 
                       l_2 * sin(a[2] - theta), 
                       l_2 * sin(a[3] + theta)]])

        return T

    def H(self, a_0, f_0):
        #TODO Fix the trig so it isnt so ass.

        theta = self.theta
        l_1 = self.l_1
        l_2 = self.l_2

        J  = np.array([[-sin(a_0[0]), -sin(a_0[1]), -sin(a_0[2]), -sin(a_0[3])],
                      [cos(a_0[0]), cos(a_0[1]), cos(a_0[2]), cos(a_0[3])],
                      [l_1 * cos(a_0[0] + theta), l_1 * cos(a_0[1] -theta), l_2 * cos(a_0[2] -theta), l_2 * cos(a_0[3] + theta)]])

        H = J * f_0
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
    

""" # Create allocation instance
alloc = Allocation()

# Test input: desired forces/torques
alloc.tau = np.array([0, 0, 0.0])  # Pure surge force (forward)

# Initial state (all thrusters at 0°, zero force)
alloc.f_0 = np.array([0.0, 0.0, 0.0, 0.0])

alloc.a_0 = np.array([0, 0, 0, 0])

alloc.Q = 1.0 * sparse.eye(3, format='csc')
alloc.omega = 1.0 * sparse.eye(4, format='csc')
alloc.rho = 0.0001

# Solve
for i in range(30):
    res = alloc.solve_thrust()

    if res and res.info.status == 'solved':
        x = res.x
        diff_f_0 = x[:4]
        diff_a_0 = x[4:8]

        alloc.f_0 = alloc.f_0 + diff_f_0
        alloc.a_0 = alloc.a_0 + diff_a_0

        
if res and res.info.status == 'solved':
    x = res.x
    delta_f = x[:4]
    delta_a = x[4:8]
    s = x[8:11]
    
    print("\n=== Solution ===")
    print(f"Δf: {delta_f}")
    print(f"Δa (deg): {np.rad2deg(delta_a)}")
    print(f"f = f_0 + Δf: {alloc.f_0 + delta_f}")
    print(f"a (deg): {np.rad2deg(alloc.a_0 + delta_a)}")
    print(f"Slack s: {s}")
    print(f"Objective: {res.info.obj_val}") """