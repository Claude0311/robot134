'''
    three DOF controller
'''

import rclpy
import numpy as np

from threeDOF.demo134     import DemoNode
from threeDOF.KinematicChain    import KinematicChain
from threeDOF.TransformHelpers  import *

#
#   Spline Helper
#
#   We could also the Segments module.  But this seemed quicker and easier?
#
def spline(t, T, p0, pf):
    p = p0 + (pf-p0) * (3*t**2/T**2 - 2*t**3/T**3)
    v =      (pf-p0) * (6*t   /T**2 - 6*t**2/T**3)
    return (p, v)

def wrap360(angle):
    return angle - 2*np.pi * np.round(angle/(2*np.pi))

#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node, q0):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())

        self.node = node

        self.print(q0)

        self.q0 = np.array(q0).reshape((-1,1))
        self.qsafe = np.array([0, -np.pi/2, np.pi/2]).reshape((-1,1))
        self.xsafe = self.chain.fkin(self.qsafe)[0]
        
        self.qR = np.array([ np.pi/4, -np.pi/4, np.pi/2]).reshape((-1,1))
        self.x1 = self.chain.fkin(self.qR)[0]

        self.qL = np.array([-np.pi/4, -np.pi/4, np.pi/2]).reshape((-1,1))
        self.x2 = self.chain.fkin(self.qL)[0]

        self.print(self.x1, self.x2)

        self.q = self.q0
        self.chain.setjoints(self.q)
        
        # self.print(self.xsafe)
        
        self.lam = 20
    
    def print(self, *argv):
        for arg in argv:
            self.node.get_logger().info(str(arg))

    def setFlip(self):
        self.q_before = self.q
        q0 = self.q[0]-np.pi if self.q[0]>0 else self.q[0]+np.pi
        q1 = -np.pi - self.q[1]
        q2 = -self.q[2]
        self.q_after = np.array([q0,q1,q2]).reshape((-1,1))

    def flip(self, t, dt, T):
        (q, qdot) = spline(t, T, self.q_before, self.q_after)
        self.q = q
        self.chain.setjoints(q)
        return (q.flatten().tolist(), qdot.flatten().tolist())

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3']

    def toSafe(self, t, dt, T):
        if t<T/2:
            (q,qdot) = spline(t,  T/2, self.q0, self.qsafe)
        else:
            (q,qdot) = spline(t-T/2,  T/2, self.qsafe, self.qR)
        return (q,qdot)
    
    def toLines(self, t, dt):
        T = 5
        if t%(2*T)<T:
            (x_desire, xddot) = spline(t%T, T, self.x1, self.x2)
        else:
            (x_desire, xddot) = spline(t%T, T, self.x2, self.x1)
        J = self.chain.Jv()
        xq = self.chain.ptip()
        qdot = np.linalg.pinv(J, 0.1) @ (xddot + self.lam * (x_desire - xq))
        q = self.q + qdot * dt
        return (q,qdot)

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        T = 10
        if t<T:
            (q,qdot) = self.toSafe(t, dt, T)
        else:
            (q,qdot) = self.toLines(t-T, dt)
        
        self.q = q
        self.chain.setjoints(self.q)
        # Return the position and velocity as python lists.
        return (q.flatten().tolist(), qdot.flatten().tolist())


#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the generator node (100Hz) for the Trajectory.
    rclpy.init(args=args)
    node = DemoNode('generator', Trajectory)

    # Spin, until interrupted or the trajectory ends.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()