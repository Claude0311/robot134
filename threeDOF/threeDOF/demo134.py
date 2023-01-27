#!/usr/bin/env python3
#
#   demo134.py
#
#   Demonstration node to interact with the HEBIs.
#
import numpy as np
import rclpy
import math

from rclpy.node         import Node
from sensor_msgs.msg    import JointState

from rclpy.parameter    import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from std_msgs.msg import Empty
#
#   Definitions
#
RATE = 100.0            # Hertz


#
#   DEMO Node Class
#
class DemoNode(Node):
    # Initialization.
    def __init__(self, name, Trajectory, simulation=True):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Create a temporary subscriber to grab the initial position.
        self.readparameters()
        simulation = self.simulation
        self.get_logger().info("Using simulation %s" % simulation)
        if simulation:
            self.position0 = [0, 0, 0]
        else:
            self.position0 = self.grabfbk()

        self.trajectory = Trajectory(self, self.position0)
        self.jointnames = self.trajectory.jointnames() #if simulation else ['one', 'two', 'three']

        self.get_logger().info("Initial positions: %r" % self.position0)

        # Create a message and publisher to send the joint commands.
        self.cmdmsg = JointState()
        if simulation:
            self.cmdpub = self.create_publisher(JointState, '/joint_states', 10)
        else:
            self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)

        # Create a subscriber to continually receive joint state messages.
        self.fbksub = self.create_subscription(
            JointState, '/joint_states', self.recvfbk, 10)

        # Create a timer to keep calculating/sending commands.
        rate       = RATE
        self.timer = self.create_timer(1/rate, self.sendcmd)
        self.t  = 0.0
        self.dt = self.timer.timer_period_ns * 1e-9
        self.get_logger().info("Sending commands with dt of %f seconds (%fHz)" %
                               (self.dt, rate))

        self.flipsub = self.create_subscription(Empty, '/flip', self.cb_flip, 1)
        self.flipping = False
        self.fliptime = 0

        self.actpos = None
        self.actvel = None
        self.acteff = None
        self.gravity_scale = 1.0
        self.collidetime = 9999
        self.skip_colli = 0.0 # when < 1, skip
        if not simulation:
            self.statessub = self.create_subscription(
                    JointState, '/joint_states', self.cb_states, 1)
            while self.actpos is None:
                rclpy.spin_once(self)
            self.get_logger().info("Initial positions: %r" % self.actpos)
            

    def cb_flip(self, msg):
        self.get_logger().info("Flipping...")
        self.fliptime = 0
        self.flipping = True
        self.trajectory.setFlip(self.actpos)

    def cb_states(self, msg):
        # Save the actual position.
        self.actpos = msg.position
        self.actvel = msg.velocity
        self.acteff = msg.effort

    def check_colli(self, q, qd, qdd):
        q_threshold = 0.03
        qd_threshold = 0.3
        qdd_threshold = 1
        for i in range(3):
            if np.abs(q[i]-self.actpos[i])>q_threshold or np.abs(qd[i]-self.actvel[i])>qd_threshold or np.abs(qdd[i]-self.acteff[i])>qdd_threshold:
                return True
        return False

    def gravity(self, pos):
        if pos is None: return (0.0,0.0,0.0)
        scale = self.gravity_scale
        (A, B, C, D) = (0.00, -0.15, -0.5, -2.75)#(0.01*scale, 0.1*scale, -0.01*scale, -1.0*scale)
        (t0, t1, t2) = list(pos)
        t1 = -t1
        t2 = -t2
        tau1 = A*math.sin(t1+t2) + B*math.cos(t1+t2) + C*math.sin(t1) + D*math.cos(t1)
        tau2 = A*math.sin(t1+t2) + B*math.cos(t1+t2)
        return (0.0, tau1, tau2)


    def readparameters(self):
        BOOL      = ParameterDescriptor(type=ParameterType.PARAMETER_BOOL)
        self.declare_parameter('simu',   descriptor=BOOL, value=False)
        self.simulation = self.get_parameter('simu').get_parameter_value().bool_value
        

    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()


    # Grab a single feedback - do not call this repeatedly.
    def grabfbk(self):
        # Create a temporary handler to grab the position.
        def cb(fbkmsg):
            self.grabpos   = list(fbkmsg.position)
            self.grabready = True

        # Temporarily subscribe to get just one message.
        sub = self.create_subscription(JointState, '/joint_states', cb, 1)
        self.grabready = False
        while not self.grabready:
            rclpy.spin_once(self)
        self.destroy_subscription(sub)

        # Return the values.
        return self.grabpos


    # Receive feedback - called repeatedly by incoming messages.
    def recvfbk(self, fbkmsg):
        # Just print the position (for now).
        # print(list(fbkmsg.position))
        pass

    # Send a command - called repeatedly by the timer.
    def sendcmd(self):
        # Build up the message and publish.
        if self.skip_colli<1:
            self.skip_colli += self.dt

        if self.collidetime < 1:
            self.collidetime += self.dt
            nan = float('nan')
            q = (nan, nan, nan)
            qdot = (nan, nan, nan)
            qdotdot = self.gravity(self.actpos)
            self.skip_colli = 0.0
            self.trajectory.q_before = np.array(self.actpos).reshape((-1, 1))

        else:

            if self.flipping and self.fliptime<10:
                self.fliptime += self.dt
                t  = self.fliptime
                dt = self.dt
                desired = self.trajectory.flip(t, dt, T=10)
                self.skip_colli = 0.0

            else:
                self.t += self.dt
                t  = self.t
                dt = self.dt
                desired = self.trajectory.evaluate(t, dt)

            if desired is None:
                self.future.set_result("Trajectory has ended")
                return
            (q, qdot) = desired
            qdotdot = self.gravity(self.actpos)

            if not self.simulation and self.skip_colli>=1 and self.check_colli(q, qdot, qdotdot):
                self.get_logger().info('Collide!!!')
                self.cb_flip('msg')
                self.collidetime = 0

        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name         = self.jointnames
        self.cmdmsg.position     = q
        self.cmdmsg.velocity     = qdot
        self.cmdmsg.effort       = qdotdot
        self.cmdpub.publish(self.cmdmsg)


#
#   Main Code
#
# def main(args=None):
#     # Initialize ROS.
#     rclpy.init(args=args)

#     # Instantiate the DEMO node.
#     node = DemoNode('demo')

#     # Spin the node until interrupted.
#     rclpy.spin(node)

#     # Shutdown the node and ROS.
#     node.shutdown()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()
