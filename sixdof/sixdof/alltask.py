'''
    three DOF controller
'''

import rclpy
import numpy as np

from sixdof.demo134     import DemoNode
from threeDOF.KinematicChain    import KinematicChain
from threeDOF.TransformHelpers  import *
from math import atan2

from std_msgs.msg import Float32MultiArray, Int8

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
        
        
        self.xtarget = [] #np.array([-0.5, -0.3, 0.0]).reshape((-1, 1))
        self.Rtarget = [] #
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

        self.phase = 0 
        # 0 -> wait for cmd
        # 1 -> move to target
        # 2 -> grap
        # 3 -> move back to normal
        # 4 -> release
        self.task = 0
        # 0 -> pick up target
        # 1 -> hit pile
        # self.settarget(None)

        self.sub = self.node.create_subscription(
            Float32MultiArray, '/do_action', self.settarget, 10)
        self.pub = self.node.create_publisher(
            Int8, '/cur_phase', 10
        )
    
    def print(self, *argv):
        for arg in argv:
            self.node.get_logger().info(str(arg))

    def settarget(self, msg=None):
        if self.phase != 0: return
        if msg is None:
            self.node.get_logger().info('msg type wrong')
            self.node.get_logger().info(str(msg.data))
            return
        data = msg.data
        if len(data)==0: return
        self.task = int(data[0])
        if self.task==0:
            self.node.get_logger().info('task: pick up tile')

            [action, index, cx, cy, tx, ty] = msg.data
            xtarget = [self.xsafe]
            Rtarget = [self.Rsafe]

            theta = atan2(ty-cy, tx-cx)
            cx = cx + 0.02 * np.cos(theta + np.pi/4)
            cy = cy + 0.02 * np.sin(theta + np.pi/4)

            xtarget.append( np.array([cx, cy, 0.02]).reshape((-1,1)) )
            Rtarget.append( Rotz(theta) @ Rotx(np.pi) )

            xtarget.append( np.array([-0.6+index*0.06, 0.3, 0.02]).reshape((-1,1)) )
            Rtarget.append( Rotz(np.pi/2) @ Rotx(np.pi) )

            self.eh = []
            self.alpha = []

            for ph in [1, 2]:
                #### calculate the eigenvector ####
                R0f = (Rtarget[ph]).T @ Rtarget[ph-1]
                w,v = np.linalg.eig( R0f )
                index = np.argmin(np.abs(w-1.))
                u = v[:,index].reshape((3,1))
                u = np.real(u)
                alpha = np.arccos((np.trace(R0f)-1)/2)
                if not np.allclose(Rote(u,-alpha),R0f):
                    alpha = -alpha
                ###################################

                self.eh.append( u )
                self.alpha.append( alpha )

            self.xtarget = xtarget
            self.Rtarget = Rtarget

            self.phase = 1

        elif self.task==2:
            [_, _, cx, cy] = data
            self.node.get_logger().info(str([cx, cy]))
            self.xtarget = np.array([cx, cy, 0.02]).reshape((-1,1))
            theta = atan2(cy, cx)
            self.Rtarget = Rotz(theta) @ Rotx(np.pi)

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
            self.phase = 1

        else: return

    def setFlip(self, pos, v0 = None):
        if pos is None: self.q_before = self.q
        else: self.q_before = np.array(pos).reshape((-1, 1)) #self.q
        # if v0 is None: self.q_dot_before = self.q_dot
        # else: self.q_dot_before = np.array(v0).reshape((-1, 1))
        q0 = self.q[0]-np.pi if self.q[0]>0 else self.q[0]+np.pi
        q1 = -np.pi - self.q[1]
        q2 = -self.q[2]
        self.q_after = np.array([q0,q1,q2]).reshape((-1,1))
        self.chain.setjoints(self.q_after)
        
        if self.x_desire is not None:
            for i in range(10):
                J = self.chain.Jv()
                xq = self.chain.ptip()
                q_dot_temp = np.linalg.pinv(J, 0.1) @ (self.x_desire - xq)
                self.q_after += q_dot_temp
                self.chain.setjoints(self.q_after)
        J = self.chain.Jv()
        self.q_dot_after = np.linalg.pinv(J, 0.1) @ self.xdot

    def flip(self, t, dt, T):
        (q_desire, qdot) = spline(t, T, self.q_before, self.q_after, vf = self.q_dot_after) #, v0 = self.q_dot_before)
        self.q = q_desire
        self.chain.setjoints(self.q)
        return (self.q.flatten().tolist(), qdot.flatten().tolist())

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5']

    def toSafe(self, t, dt, T):
        (q,qdot) = spline(t,  T, self.q0, self.qsafe)
        return (q,qdot)

    def movement_picktile(self, t_total, dt, T):
        t = t_total-self.t0
        if self.phase in [1, 3]:
            xtarget_higher = self.xtarget[1]*1
            xfrom_higher = self.xtarget[0]*1
            xtarget_higher[2] = 0.13
            xfrom_higher[2] = 0.13
            if self.phase == 1:
                if t<2/3*T:
                    (x_desire, xddot) = spline(t, 2/3*T, self.xtarget[0], xtarget_higher)
                    (theta_desire, wd) = spline(t, 2/3*T, 0, self.alpha[0])
                else:
                    (x_desire, xddot) = spline(t-2/3*T, 1/3*T, xtarget_higher, self.xtarget[1])
                    (theta_desire, wd) = spline(t-2/3*T, 1/3*T, self.alpha[0], self.alpha[0])
            else:
                if t<1/4*T:
                    (x_desire, xddot) = spline(t, 1/4*T, self.xtarget[0], xfrom_higher)
                    (theta_desire, wd) = spline(t, 1/4*T, 0, 0)
                elif t<3/4*T:
                    (x_desire, xddot) = spline(t-1/4*T, 1/2*T, xfrom_higher, xtarget_higher)
                    (theta_desire, wd) = spline(t-1/4*T, 1/2*T, 0, self.alpha[0])
                else:
                    (x_desire, xddot) = spline(t-3/4*T, 1/4*T, xtarget_higher, self.xtarget[1])
                    (theta_desire, wd) = spline(t-3/4*T, 1/4*T, self.alpha[0], self.alpha[0])
        
            Jv = self.chain.Jv()
            Jw = self.chain.Jw()
            Jv_inv = np.linalg.pinv(Jv, 0.1)
            Jw_inv = np.linalg.pinv(Jw, 0.1)

            xq = self.chain.ptip()

            R0 = self.Rtarget[0]
            wd = R0 @ self.eh[0] * wd
            

            self.Rd = R0 @ Rote(self.eh[0], theta_desire)
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
            tmp_qtarget = self.q_target * 1
            tmp_qtarget[1] = 0.9*self.q_target[1] + 0.1*self.qsafe[1]

            if t < 1/4*T:
                (q, qdot) = spline(t, 1/4*T, self.q_target, tmp_qtarget)
            else:
                (q, qdot) = spline(t-1/4*T, 3/4*T, tmp_qtarget, self.qsafe)
            return (q,qdot)
            
    def picktile_evaluate(self, t, dt):
        T = 5
        loose = -0.8
        tight = -1.4
        gripper_theta = loose

        if self.phase==1:
            (q,qdot) = self.movement_picktile(t, dt,  T)

            if t>self.t0+T:
                self.phase = 2
                self.t0 = t
                self.xtarget.pop(0)
                self.Rtarget.pop(0)
                self.eh.pop(0)
                self.alpha.pop(0)
                self.q_target = self.q
                self.Rd = None
                self.x_desire = None

        elif self.phase==2:
            gripper_theta = loose + (t-self.t0)/5 * (tight-loose)
            q = self.q
            qdot = self.q_dot
            if t>self.t0+5:
                self.phase = 3
                self.t0 = t

        elif self.phase==3:
            (q,qdot) = self.movement_picktile(t, dt,  T)
            gripper_theta = tight

            if t>self.t0+T:
                self.phase = 4 
                self.t0 = t
                self.xtarget.pop(0)
                self.Rtarget.pop(0)
                self.eh.pop(0)
                self.alpha.pop(0)
                self.q_target = self.q
                self.Rd = None
                self.x_desire = None

        elif self.phase==4:
            gripper_theta = tight + (t-self.t0)/5* (loose-tight)
            q = self.q
            qdot = self.q_dot
            if t>self.t0+5:
                self.t0 = t
                self.phase = 5

        elif self.phase==5:
            (q, qdot) = self.movement_picktile(t, dt,  T)

            if t>self.t0+5:
                self.t0 = t
                self.phase = 0
                mymsg = Int8()
                mymsg.data = 0
                self.pub.publish(mymsg)

        return (q, qdot, gripper_theta)

    def movement_hitpile(self, t, dt, T):
        if self.phase == 1:
            xtarget_higher = self.xtarget*1
            xtarget_higher[2] = 0.13
            if t-self.t0<2/3*T:
                (x_desire, xddot) = spline(t-self.t0, 2/3*T, self.xsafe, xtarget_higher)
                (theta_desire, wd) = spline(t-self.t0, 2/3*T, 0, self.alpha)
            else:
                (x_desire, xddot) = spline(t-self.t0-2/3*T, 1/3*T, xtarget_higher, self.xtarget)
                (theta_desire, wd) = spline(t-self.t0-2/3*T, 1/3*T, self.alpha, self.alpha)
        
            Jv = self.chain.Jv()
            Jw = self.chain.Jw()
            Jv_inv = np.linalg.pinv(Jv, 0.1)
            Jw_inv = np.linalg.pinv(Jw, 0.1)

            xq = self.chain.ptip()

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
            # qdot = Jw_inv @ wd 
            # J = np.vstack((Jv, Jw))
            # qdot = np.linalg.pinv(J, 0.1) @ pd
            q = self.q + qdot * dt
            return (q,qdot)

        else:
            (q, qdot) = spline(t-self.t0, T, self.q_target, self.qsafe)
            return (q,qdot)

    def hitpile_evaluate(self, t, dt):
        T = 5
        loose = -0.8
        tight = -1.4
        gripper_theta = loose

        if self.phase==1:
            (q,qdot) = self.movement_hitpile(t, dt,  T)

            if t>self.t0+T:
                self.phase = 2
                self.t0 = t
                self.xtarget = None
                self.q_target = self.q
                self.Rd = None
                self.x_desire = None

        elif self.phase==2:
            q = self.q
            qdot = self.q_dot * 0
            t_inner = t-self.t0

            if t_inner<1:
                q = self.q
                qdot = self.q_dot
                gripper_theta = loose + (t_inner) * (tight-loose)

            elif 1<=t_inner<3:
                qdot[1, 0] = - 0.1 * np.sin((t_inner-1)/2 * np.pi)
                qdot[3,0] = - 0.5 * np.sin((t_inner-1)/2 * np.pi)
                q += qdot * dt
                gripper_theta = tight

            elif 3<=t_inner<4:
                q = self.q
                qdot = self.q_dot
                gripper_theta = tight - (t_inner-3)*(tight-loose)
            
            else: #4~6
                qdot[3,0] = + 0.5 * np.sin((t_inner-4)/2 * np.pi)
                q += qdot * dt
                gripper_theta = loose

            if t>self.t0+6:
                self.phase = 3
                self.t0 = t
                self.q_target = self.q

        elif self.phase==3:
            (q,qdot) = self.movement_hitpile(t, dt,  T)

            if t>self.t0+T:
                self.phase = 4 
                self.t0 = t

        elif self.phase==4:
            q = self.q
            qdot = self.q_dot
            if t>self.t0+5:
                self.t0 = t
                self.phase = 0
                mymsg = Int8()
                mymsg.data = 0
                self.pub.publish(mymsg)
        
        return (q, qdot, gripper_theta)

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        T = 5
        nan = float('nan')
        loose = -0.8
        tight = -1.4
        gripper_theta = loose
        gripper_v = nan

        if t<T:
            (q,qdot) = self.toSafe(t, dt,  T)
            self.t0 = t

            if t>=T-dt:
                self.phase = 0
                mymsg = Int8()
                mymsg.data = 0
                self.pub.publish(mymsg)

        elif self.phase==0:
            self.t0 = t
            q = self.q
            qdot = self.q_dot

        elif self.task==0:
            (q, qdot, gripper_theta) = self.picktile_evaluate(t, dt)

        elif self.task==2:
            (q, qdot, gripper_theta) = self.hitpile_evaluate(t, dt)

        
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
    node = DemoNode('alltask', Trajectory)

    # Spin, until interrupted or the trajectory ends.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()