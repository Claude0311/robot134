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

#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node, q0=[0,0,0]):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())

        self.q0 = np.array(q0)
        self.q1 = -np.pi/2 * np.sign(self.q0)
        self.q1[self.q1==0] = -np.pi/2 # in case it's exactly 0
        self.offset = (self.q0 + self.q1)/2
        self.amp = (self.q0 - self.q1)/2

        self.qR = np.array([np.pi/4,np.pi/4,-np.pi/2])
        self.qL = np.array([-np.pi/4,np.pi/4,-np.pi/2])
        self.qM = np.array([0,np.pi/2,-np.pi/2])

        self.q = self.q0
        self.chain.setjoints(self.q)
        
        self.lam = 20


    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        # q    = self.offset + self.amp * np.cos(t)
        # qdot =             - self.amp * np.sin(t)
        if t<3: #0 -> M
            (q,qdot) = spline(t,  3, self.q0, self.qM)
        elif t%12<3: # L -> M
            (q,qdot) = spline(t%3,3, self.qL, self.qM)
        elif t%12<6: # M -> R
            (q,qdot) = spline(t%3,3, self.qM, self.qR)
        elif t%12<9: # R -> M
            (q,qdot) = spline(t%3,3, self.qR, self.qM)
        else: # M -> L
            (q,qdot) = spline(t%3,3, self.qM, self.qL)
        
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