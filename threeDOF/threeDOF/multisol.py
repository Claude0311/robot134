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

        self.sol1 = np.array([0.0, -np.pi/4, np.pi/2])
        self.sol2 = np.array([np.pi, -3*np.pi/4, -np.pi/2])

        self.q = self.q0
        self.chain.setjoints(self.q)
        
        self.lam = 20


    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        if t<3: #0 -> 1
            (q,qdot) = spline(t,  3, self.q0, self.sol1)
        elif t%6<3: # 2 -> 1
            (q,qdot) = spline(t%3,3, self.sol2, self.sol1)
        else: # 1 -> 2
            (q,qdot) = spline(t%3,3, self.sol1, self.sol2)
        
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