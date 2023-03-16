'''
    three DOF controller
'''

import rclpy
import numpy as np

from sixdof.demo134     import DemoNode
from threeDOF.KinematicChain    import KinematicChain
from threeDOF.TransformHelpers  import *
from math import atan2

from std_msgs.msg import Float32MultiArray

#
#   Spline Helper
#
#   We could also the Segments module.  But this seemed quicker and easier?
#
def spline(t, T, p0, pf, v0 = None, vf = None):
    if v0 is None: v0 = 0*p0 #same shape
    if vf is None: vf = 0*pf
    a = p0
    b = v0
    c = 3*(pf-p0)/T**2 - vf/T    - 2*v0/T
    d = -2*(pf-p0)/T**3 + vf/T**2 +   v0/T**2

    # p = p0 + (pf-p0) * (3*t**2/T**2 - 2*t**3/T**3)
    # v =      (pf-p0) * (6*t   /T**2 - 6*t**2/T**3)
    p = a + b * t +   c * t**2 +   d * t**3
    v =     b     + 2*c * t    + 3*d * t**2
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

        # ignore the gripper controller
        ptr = np.array([0,1,3,4,5])
        self.q0 = np.array(q0)[ptr].reshape((-1,1))
        self.node.get_logger().info(str(q0))
        self.qsafe = np.array([0.0, -np.pi/2, np.pi/8*5, -np.pi/8*3, 0.0]).reshape((-1,1))
        safepos = self.chain.fkin(self.qsafe)
        self.xsafe = safepos[0]
        self.Rsafe = safepos[1]
        
        
        self.xtarget = None #np.array([-0.5, -0.3, 0.0]).reshape((-1, 1))
        self.Rtarget = None #
        self.eh = None      #eigenvector for rotation
        self.alpha = None   #angle to rotate
        self.Rerr = None
        self.Rd = None

        self.q = self.q0
        self.chain.setjoints(self.q)
        
        # self.print(self.xsafe)
        
        self.xdot = np.array([0.0, 0.0, 0.0]).reshape((-1, 1))
        self.q_dot_after = np.array([0.0, 0.0, 0.0]).reshape((-1, 1))
        self.x_desire = None
        self.lam = 10
        self.t0 = 0
        
        # for flipping
        self.og_theta4 = None

        self.phase = 0 
        # 0 -> wait for cmd
        # 1 -> move to target
        # 2 -> grap
        # 3 -> move back to normal
        # 4 -> release
        # self.settarget(None)

        self.sub = self.node.create_subscription(
            Float32MultiArray, '/target', self.settarget, 10)
    
    def print(self, *argv):
        for arg in argv:
            self.node.get_logger().info(str(arg))

    def settarget(self, msg=None):
        inbounds = True 
        if self.phase == 0 or self.phase==7:
            if msg is None or len(msg.data)!=4:
                # can call settaarget() in __init__() for test
                self.xtarget = np.array([-0.5, -0.1, 0.1]).reshape((-1,1))
                self.Rtarget = Rotx(np.pi) @ Rotz(0)
            else:
                data = msg.data
                theta = atan2(data[3]-data[1], data[2]-data[0])
                print("theta: ", theta) 
                # force forward grip by asserting negative theta
                self.xtarget = np.array([data[0],data[1], 0.01]).reshape((-1,1))
                # if outside of bounds, wait in phase0 until inside bounds
                print("distance: ", np.linalg.norm(self.xtarget))
                if (np.linalg.norm(self.xtarget) > 0.78):
                    print("WE ARE OUTSIDE OF THE REACHABLE RANGE")
                    inbounds = False
                # change the tip frame (try fully flat at first
                self.Rtarget = Rotz(theta) @ Rotx(np.pi)
                if self.phase==7:
                    if (theta > 0):
                        theta = theta - np.pi
                    print("self.xtarget: ", self.xtarget)
                    self.Rtarget = Rotz(theta) @ Rotx(5/8 * np.pi)
                    print("self.Rtarget: ", self.Rtarget)

            #### calculate the eigenvector ####
            R0f = (self.Rtarget).T @ self.Rsafe
            w,v = np.linalg.eig( R0f )
            index = np.argmin(np.abs(w-1.))
            u = v[:,index].reshape((3,1))
            u = np.real(u)
            alpha = np.arccos((np.trace(R0f)-1)/2)
            if not np.allclose(Rote(u,-alpha),R0f):
                alpha = -alpha
            ###################################

            self.eh = u

            self.alpha = alpha
            
            if inbounds:
                if self.phase==0: self.phase = 1
                elif self.phase==7: self.phase = 8
            else:
                print("back to 0")
                self.phase = 0

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5']

    def toSafe(self, t, dt, T):
        (q,qdot) = spline(t,  T, self.q0, self.qsafe)
        return (q,qdot)

    def toLines(self, t, dt, T):
        if self.phase in [1, 8]:
            xtarget_higher = self.xtarget*1
            xtarget_higher[2] = 0.13
            if self.phase == 8:
                # decrease z, increase y
                xtarget_higher[0] += 0.05
                xtarget_higher[2] = 0.1

            xq = self.chain.ptip()
            if self.phase == 1:
                x0 = self.xsafe
            else:
                x0 = xq 
            # if not phase 1, our starting position is xq 
            if t-self.t0<2/3*T:
                (x_desire, xddot) = spline(t-self.t0, 2/3*T, x0, xtarget_higher)
                (theta_desire, wd) = spline(t-self.t0, 2/3*T, 0, self.alpha)
            else:
                (x_desire, xddot) = \
                        spline(t-self.t0-2/3*T, 1/3*T, xtarget_higher, self.xtarget)
                (theta_desire, wd) = spline(t-self.t0-2/3*T, 1/3*T, self.alpha, self.alpha)
            
            Jv = self.chain.Jv()
            Jw = self.chain.Jw()
            Jv_inv = np.linalg.pinv(Jv, 0.1)
            Jw_inv = np.linalg.pinv(Jw, 0.1)


            R0 = self.Rsafe
            wd = R0 @ self.eh * wd
            

            self.Rd = R0 @ Rote(self.eh, theta_desire)
            self.x_desire = x_desire

            self.xdot = xddot

            if self.Rerr is not None:
                xddot +=  self.lam * self.Rerr[:3]
                wd += self.lam * self.Rerr[3:]
            pd = np.vstack((xddot,wd))
            qdot = Jv_inv @ xddot + nullspace(Jv, Jv_inv) @ Jw_inv @ wd 
            q = self.q + qdot * dt
            return (q,qdot)

        else:
            # return to safe
            (q, qdot) = spline(t-self.t0, T, self.q_target, self.qsafe)
            print("returning to safe")
            return (q,qdot)

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        T = 5
        nan = float('nan')
        loose = -0.8
        tight = -1.4
        gripper_theta = loose
        gripper_v = nan
        
        print("phase: ", self.phase)
        if t<T:
            (q,qdot) = self.toSafe(t, dt,  T)
            self.t0 = t

        elif self.phase==0:
            self.t0 = t
            q = self.q
            qdot = self.q_dot

        # move to target
        elif self.phase==1:
            (q,qdot) = self.toLines(t, dt,  T)

            if t>self.t0+T:
                self.phase = 2
                self.t0 = t
                self.xtarget = None
                self.q_target = self.q
                self.Rd = None
                self.x_desire = None 
                self.og_theta4 = q[4]*1
        # rotate tile to be parallel
        elif self.phase==2:
            T = 3
            # tighten gripper
            gripper_theta = loose + (t - self.t0)/T * (tight - loose)

            q = self.q
            qdot = self.q_dot
            print("grab tile")
            if t>self.t0+T:
                self.phase = 3
                self.t0 = t
        elif self.phase==3:
            T = 3
            q = self.q
            qdot = self.q_dot
            gripper_theta = tight
            # lift straight up
            print("lifting tile straight up")
            print("dt: ", dt)
            q[1] -= dt * (1.5 / 180) * np.pi 

            if t>self.t0+T:
                self.phase = 4
                self.t0 = t
                self.og_theta4 = q[4] * 1
                self.q_target = self.q
        elif self.phase==4:
            T = 3
            q = self.q
            qdot = self.q_dot
            gripper_theta = tight 
            # aligning
            distance = \
              (self.og_theta4 - (-45/180) * np.pi) % np.pi  
            distance = -distance
            q[4] = (t - self.t0)/T * distance + self.og_theta4  
            
            if t>self.t0+T:
                self.phase = 5
                self.t0 = t
                self.og_theta4 = q[4]*1
                self.q_target = self.q
        elif self.phase==5:
            T = 1
            q = self.q
            qdot = self.q_dot
            gripper_theta = loose
            
            # drop it
            print("dropping tile")
            if t>self.t0+T:
                self.phase = 6
                self.t0 = t
        elif self.phase==6:
            # move back to safe so we can see the message
            T = 5
            (q,qdot) = self.toLines(t, dt,  T)
            
            if t>self.t0+T:
                self.phase = 7
                self.t0 = t
                self.xtarget = None
                self.q_target = self.q
                self.Rd = None
                self.x_desire = None 
        # Now for actual flipping
        # stall until message of xtarget is received
        elif self.phase == 7:
            self.t0 = t
            q = self.q
            qdot = self.q_dot
        elif self.phase==8:
            # go back to tile at an angle
            T = 5
            (q,qdot) = self.toLines(t, dt,  T)

            if t>self.t0+T:
                self.phase = 9
                self.t0 = t
                self.xtarget = None
                self.q_target = self.q
                self.Rd = None
                self.x_desire = None 
                self.og_theta4 = q[4]*1
        elif self.phase==9:
            T = 3
            q = self.q
            qdot = self.q_dot
            # tighten gripper
            print("gripping tile")
            gripper_theta = loose + (t - self.t0)/T * (tight - loose)
            
            if t>self.t0+T:
                self.phase = 10
                self.t0 = t
        elif self.phase==10:
            T = 3
            q = self.q
            qdot = self.q_dot
            gripper_theta = tight 
            # lift it 
            print("lifting arm angle")
            q[1] -= dt * (2 / 180) * np.pi 
            q[3] += dt * (5 / 180) * np.pi * np.sign(q[3]) 
            if t>self.t0+T:
                self.phase = 11
                self.t0 = t
                self.q_target = self.q
        # flip it    
        elif self.phase==11:
            T = 3
            q = self.q
            qdot = self.q_dot
            gripper_theta = tight  
            print("flipping")
            q[4] = (t - self.t0)/T * np.pi + self.og_theta4 
            if t>self.t0+T:
                self.phase = 12 
                self.t0 = t
                # arm has been rotated, so q_target has been changed
                self.q_target = self.q
        # drop it
        elif self.phase==12:
            T = 1 
            q = self.q
            qdot = self.q_dot
            # tighten gripper
            gripper_theta = loose 
            print("dropping")
            if t>self.t0+T:
                self.phase = 13
                self.t0 = t
        # back to safe
        elif self.phase==13:
            T = 5
            (q, qdot) = self.toLines(t, dt, T)
            gripper_theta = loose

            if t>self.t0+T:
                self.t0 = t
                self.phase = 0 if self.xtarget is None else 1
        
        self.q = q
        self.chain.setjoints(self.q)
        self.q_dot = qdot

        if self.Rd is not None and self.x_desire is not None:
            self.Rerr = np.vstack(
                (ep(self.x_desire,self.chain.ptip()), 
                eR(self.Rd,self.chain.Rtip()))
            )

        q_return = q.flatten().tolist()
        q_return.insert(2, gripper_theta)
        qdot_return = qdot.flatten().tolist()
        qdot_return.insert(2, gripper_v)
        # Return the position and velocity as python lists.
        return (q_return, qdot_return)


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
